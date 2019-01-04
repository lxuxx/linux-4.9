/*
 * Crypto driver for the Aspeed SoC
 *
 * Copyright (C) ASPEED Technology Inc.
 * Ryan Chen <ryan_chen@aspeedtech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "aspeed-crypto.h"

#define ASPEED_CIPHER_DEBUG

#ifdef ASPEED_CIPHER_DEBUG
//#define CIPHER_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define CIPHER_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define CIPHER_DBG(fmt, args...)
#endif

static int aspeed_crypto_sk_handle_queue(struct aspeed_crypto_dev *crypto_dev,
		struct crypto_async_request *new_areq)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct crypto_async_request *areq, *backlog;
	struct aspeed_cipher_ctx *ctx;
	unsigned long flags;
	int err, ret = 0;

	CIPHER_DBG("\n");
	spin_lock_irqsave(&sk_engine->lock, flags);
	if (new_areq)
		ret = crypto_enqueue_request(&sk_engine->queue, new_areq);
	if (sk_engine->flags & CRYPTO_FLAGS_BUSY) {
		spin_unlock_irqrestore(&sk_engine->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&sk_engine->queue);
	areq = crypto_dequeue_request(&sk_engine->queue);
	if (areq)
		sk_engine->flags |= CRYPTO_FLAGS_BUSY;
	spin_unlock_irqrestore(&sk_engine->lock, flags);

	if (!areq)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);

	ctx = crypto_tfm_ctx(areq->tfm);
	sk_engine->is_async = (areq != new_areq);
	sk_engine->areq = areq;

	err = ctx->start(crypto_dev);


	// sk_engine->sk_req = skcipher_request_cast(areq);
	// err = aspeed_crypto_skcipher_trigger(crypto_dev);

	return (sk_engine->is_async) ? ret : err;
}


static inline int aspeed_sk_wait_for_data_ready(struct aspeed_crypto_dev *crypto_dev,
		aspeed_crypto_fn_t resume)
{
#ifdef CONFIG_CRYPTO_DEV_ASPEED_SK_INT
	CIPHER_DBG("\n");

	return -EINPROGRESS;
#else
	CIPHER_DBG("\n");

	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_CRYPTO_BUSY);
	return resume(crypto_dev);
#endif
}

static int aspeed_sk_complete(struct aspeed_crypto_dev *crypto_dev, int err)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct skcipher_request *req = skcipher_request_cast(sk_engine->areq);
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));

	CIPHER_DBG("\n");
	if (ctx->enc_cmd & HACE_CMD_IV_REQUIRE) {
		if (ctx->enc_cmd & HACE_CMD_DES_SELECT)
			memcpy(req->iv, ctx->cipher_key + 8, 8);
		else
			memcpy(req->iv, ctx->cipher_key, 16);
	}
	sk_engine->flags &= ~CRYPTO_FLAGS_BUSY;
	if (sk_engine->is_async)
		req->base.complete(&req->base, err);

	aspeed_crypto_sk_handle_queue(crypto_dev, NULL);

	return err;
}

static int aspeed_sk_sg_transfer(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct skcipher_request *req = skcipher_request_cast(sk_engine->areq);
	struct crypto_skcipher *cipher = crypto_skcipher_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(cipher);

	CIPHER_DBG("\n");
	if (req->src == req->dst) {
		dma_unmap_sg(crypto_dev->dev, req->src, ctx->src_nents, DMA_BIDIRECTIONAL);
	} else {
		dma_unmap_sg(crypto_dev->dev, req->src, ctx->src_nents, DMA_TO_DEVICE);
		dma_unmap_sg(crypto_dev->dev, req->dst, ctx->dst_nents, DMA_FROM_DEVICE);
	}

	return aspeed_sk_complete(crypto_dev, 0);
}

static int aspeed_sk_cpu_transfer(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct skcipher_request *req = skcipher_request_cast(sk_engine->areq);
	struct crypto_skcipher *cipher = crypto_skcipher_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(cipher);
	struct scatterlist *out_sg = req->dst;
	int nbytes = 0;
	int err = 0;

	CIPHER_DBG("\n");
	nbytes = sg_copy_from_buffer(out_sg, ctx->dst_nents, sk_engine->cipher_addr, req->cryptlen);
	if (!nbytes) {
		printk("nbytes %d req->cryptlen %d\n", nbytes, req->cryptlen);
		err = -EINVAL;
	}
	return aspeed_sk_complete(crypto_dev, err);
}

static int aspeed_sk_dma_start(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct skcipher_request *req = skcipher_request_cast(sk_engine->areq);
	struct crypto_skcipher *cipher = crypto_skcipher_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(cipher);

	CIPHER_DBG("\n");
	CIPHER_DBG("req->cryptlen %d , nb_in_sg %d, nb_out_sg %d \n", req->cryptlen, ctx->src_nents, ctx->dst_nents);
	if (req->dst == req->src) {
		if (!dma_map_sg(crypto_dev->dev, req->src, 1, DMA_BIDIRECTIONAL)) {
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(src) error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if (!dma_map_sg(crypto_dev->dev, req->src, 1, DMA_TO_DEVICE)) {
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(src) error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		if (!dma_map_sg(crypto_dev->dev, req->dst, 1, DMA_FROM_DEVICE)) {
			dma_unmap_sg(crypto_dev->dev, req->dst, 1, DMA_FROM_DEVICE);
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(dst) error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	}
#ifdef CONFIG_CRYPTO_DEV_ASPEED_SK_INT
	sk_engine->resume = aspeed_sk_sg_transfer;
#endif
	aspeed_crypto_write(crypto_dev, sg_dma_address(req->src), ASPEED_HACE_SRC);
	aspeed_crypto_write(crypto_dev, sg_dma_address(req->dst), ASPEED_HACE_DEST);

	aspeed_crypto_write(crypto_dev, req->cryptlen, ASPEED_HACE_DATA_LEN);
	aspeed_crypto_write(crypto_dev, ctx->enc_cmd, ASPEED_HACE_CMD);
	return aspeed_sk_wait_for_data_ready(crypto_dev, aspeed_sk_sg_transfer);
}

static int aspeed_sk_cpu_start(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct skcipher_request *req = skcipher_request_cast(sk_engine->areq);
	struct crypto_skcipher *cipher = crypto_skcipher_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(cipher);
	struct scatterlist *in_sg = req->src;
	int nbytes = 0;


	CIPHER_DBG("\n");
	nbytes = sg_copy_to_buffer(in_sg, ctx->src_nents, sk_engine->cipher_addr, req->cryptlen);
	CIPHER_DBG("copy nbytes %d, req->cryptlen %d , nb_in_sg %d, nb_out_sg %d \n", nbytes, req->cryptlen, ctx->src_nents, ctx->dst_nents);
	if (!nbytes) {
		printk("nbytes error \n");
		return -EINVAL;
	}
#ifdef CONFIG_CRYPTO_DEV_ASPEED_SK_INT
	sk_engine->resume = aspeed_sk_cpu_transfer;
#endif
	aspeed_crypto_write(crypto_dev, sk_engine->cipher_dma_addr, ASPEED_HACE_SRC);
	aspeed_crypto_write(crypto_dev, sk_engine->cipher_dma_addr, ASPEED_HACE_DEST);
	aspeed_crypto_write(crypto_dev, req->cryptlen, ASPEED_HACE_DATA_LEN);
	aspeed_crypto_write(crypto_dev, ctx->enc_cmd, ASPEED_HACE_CMD);

	return aspeed_sk_wait_for_data_ready(crypto_dev, aspeed_sk_cpu_transfer);
}

static int aspeed_sk_g6_start(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct skcipher_request *req = skcipher_request_cast(sk_engine->areq);
	struct crypto_skcipher *cipher = crypto_skcipher_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(cipher);
	struct aspeed_sg_list *src_list, *dst_list;
	dma_addr_t src_dma_addr, dst_dma_addr;
	struct scatterlist *s;
	int i;

	CIPHER_DBG("\n");

	ctx->enc_cmd |= HACE_CMD_DES_SG_CTRL | HACE_CMD_SRC_SG_CTRL;

	if (req->dst == req->src) {
		if (!dma_map_sg(crypto_dev->dev, req->src, ctx->src_nents, DMA_BIDIRECTIONAL)) {
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(src) error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if (!dma_map_sg(crypto_dev->dev, req->src, ctx->src_nents, DMA_TO_DEVICE)) {
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(src) error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		if (!dma_map_sg(crypto_dev->dev, req->dst, ctx->dst_nents, DMA_FROM_DEVICE)) {
			dma_unmap_sg(crypto_dev->dev, req->dst, ctx->dst_nents, DMA_FROM_DEVICE);
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(dst) error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	}

	src_list = (struct aspeed_sg_list *) sk_engine->cipher_addr;
	src_dma_addr = sk_engine->cipher_dma_addr;
	for_each_sg(req->src, s, ctx->src_nents, i) {
		src_list[i].phy_addr = sg_dma_address(s);
		src_list[i].len = s->length;
	}
	src_list[ctx->src_nents - 1].len |= BIT(31); //TODO
	if (req->dst == req->src) {
		dst_dma_addr = src_dma_addr;
	} else {
		dst_list = (struct aspeed_sg_list *) sk_engine->dst_sg_addr;
		dst_dma_addr = sk_engine->dst_sg_dma_addr;
		for_each_sg(req->dst, s, ctx->dst_nents, i) {
			dst_list[i].phy_addr = sg_dma_address(s);
			dst_list[i].len = s->length;
		}
		dst_list[ctx->dst_nents - 1].len |= BIT(31);
	}

#ifdef CONFIG_CRYPTO_DEV_ASPEED_SK_INT
	sk_engine->resume = aspeed_sk_sg_transfer;
#endif
	aspeed_crypto_write(crypto_dev, src_dma_addr, ASPEED_HACE_SRC);
	aspeed_crypto_write(crypto_dev, dst_dma_addr, ASPEED_HACE_DEST);
	aspeed_crypto_write(crypto_dev, req->cryptlen, ASPEED_HACE_DATA_LEN);
	aspeed_crypto_write(crypto_dev, ctx->enc_cmd, ASPEED_HACE_CMD);

	return aspeed_sk_wait_for_data_ready(crypto_dev, aspeed_sk_sg_transfer);
}

int aspeed_crypto_skcipher_trigger(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct skcipher_request *req = skcipher_request_cast(sk_engine->areq);
	struct crypto_skcipher *cipher = crypto_skcipher_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(cipher);

	CIPHER_DBG("\n");
	//for enable interrupt
#ifdef CONFIG_CRYPTO_DEV_ASPEED_SK_INT
	ctx->enc_cmd |= HACE_CMD_ISR_EN;
#endif
	aspeed_crypto_write(crypto_dev, ctx->cipher_key_dma, ASPEED_HACE_CONTEXT);
	ctx->dst_nents = sg_nents(req->dst);
	ctx->src_nents = sg_nents(req->src);
	if ((ctx->dst_nents == 1) && (ctx->src_nents == 1))
		return aspeed_sk_dma_start(crypto_dev);
	if (crypto_dev->version == 6)
		return aspeed_sk_g6_start(crypto_dev);

	return aspeed_sk_cpu_start(crypto_dev);
}

static int aspeed_rc4_crypt(struct skcipher_request *req, u32 cmd)
{
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;

	CIPHER_DBG("\n");

	cmd |= HACE_CMD_RI_WO_DATA_ENABLE |
	       HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	ctx->enc_cmd = cmd;

	return aspeed_crypto_sk_handle_queue(crypto_dev, &req->base);
}

static int aspeed_rc4_setkey(struct crypto_skcipher *cipher, const u8 *in_key,
			     unsigned int key_len)
{
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(cipher);
	int i, j = 0, k = 0;
	u8 *rc4_key = ctx->cipher_key;

	CIPHER_DBG("keylen : %d : %s  \n", key_len, in_key);

	*(u32 *)(ctx->cipher_key + 0) = 0x0;
	*(u32 *)(ctx->cipher_key + 4) = 0x0;
	*(u32 *)(ctx->cipher_key + 8) = 0x0001;

	for (i = 0; i < 256; i++)
		rc4_key[16 + i] = i;

	for (i = 0; i < 256; i++) {
		u32 a = rc4_key[16 + i];
		j = (j + in_key[k] + a) & 0xff;
		rc4_key[16 + i] = rc4_key[16 + j];
		rc4_key[16 + j] = a;
		if (++k >= key_len)
			k = 0;
	}

	ctx->key_len = 256;

	return 0;
}

static int aspeed_rc4_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_rc4_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_RC4);
}

static int aspeed_rc4_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_rc4_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_RC4);
}


static int aspeed_des_crypt(struct skcipher_request *req, u32 cmd)
{
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;

	CIPHER_DBG("\n");

	if (req->iv && (cmd & HACE_CMD_IV_REQUIRE))
		memcpy(ctx->cipher_key + 8, req->iv, 8);

	cmd |= HACE_CMD_DES_SELECT | HACE_CMD_RI_WO_DATA_ENABLE | HACE_CMD_DES |
	       HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE ;

	ctx->enc_cmd = cmd;

	return aspeed_crypto_sk_handle_queue(crypto_dev, &req->base);
}

static int aspeed_des_setkey(struct crypto_skcipher *cipher, const u8 *key,
			     unsigned int keylen)
{
	struct crypto_tfm *tfm = crypto_skcipher_tfm(cipher);
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(cipher);
	u32 tmp[DES_EXPKEY_WORDS];

	CIPHER_DBG("bits : %d : \n", keylen);

	if ((keylen != DES_KEY_SIZE) && (keylen != 2 * DES_KEY_SIZE) && (keylen != 3 * DES_KEY_SIZE)) {
		crypto_skcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		printk("keylen fail %d bits \n", keylen);
		return -EINVAL;
	}

	if (keylen == DES_KEY_SIZE) {
		if (!des_ekey(tmp, key) &&
		    (tfm->crt_flags & CRYPTO_TFM_REQ_WEAK_KEY)) {
			tfm->crt_flags |= CRYPTO_TFM_RES_WEAK_KEY;
			return -EINVAL;
		}
	}

	memcpy(ctx->cipher_key + 16, key, keylen);
	ctx->key_len = keylen;

	return 0;
}

static int aspeed_tdes_ctr_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES);
}

static int aspeed_tdes_ctr_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES);
}

static int aspeed_tdes_ofb_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES);
}

static int aspeed_tdes_ofb_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES);
}

static int aspeed_tdes_cfb_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES);
}

static int aspeed_tdes_cfb_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES);
}

static int aspeed_tdes_cbc_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES);
}

static int aspeed_tdes_cbc_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES);
}

static int aspeed_tdes_ecb_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES);
}

static int aspeed_tdes_ecb_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES);
}

static int aspeed_des_ctr_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES);
}

static int aspeed_des_ctr_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES);
}

static int aspeed_des_ofb_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES);
}

static int aspeed_des_ofb_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES);
}

static int aspeed_des_cfb_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES);
}

static int aspeed_des_cfb_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES);
}

static int aspeed_des_cbc_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES);
}

static int aspeed_des_cbc_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES);
}

static int aspeed_des_ecb_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES);
}

static int aspeed_des_ecb_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES);
}

static int aspeed_aes_crypt(struct skcipher_request *req, u32 cmd)
{
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(crypto_skcipher_reqtfm(req));
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;

	if (req->iv && (cmd & HACE_CMD_IV_REQUIRE))
		memcpy(ctx->cipher_key, req->iv, 16);

	cmd |= HACE_CMD_AES_SELECT | HACE_CMD_RI_WO_DATA_ENABLE |
	       HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	switch (ctx->key_len) {
	case AES_KEYSIZE_128:
		cmd |= HACE_CMD_AES128;
		break;
	case AES_KEYSIZE_192:
		cmd |= HACE_CMD_AES192;
		break;
	case AES_KEYSIZE_256:
		cmd |= HACE_CMD_AES256;
		break;
	}

	ctx->enc_cmd = cmd;

	return aspeed_crypto_sk_handle_queue(crypto_dev, &req->base);
}

static int aspeed_aes_setkey(struct crypto_skcipher *cipher, const u8 *key,
			     unsigned int keylen)
{
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(cipher);
	struct crypto_aes_ctx gen_aes_key;

	CIPHER_DBG("bits : %d \n", (keylen * 8));

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256) {
		crypto_skcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	crypto_aes_expand_key(&gen_aes_key, key, keylen);
	memcpy(ctx->cipher_key + 16, gen_aes_key.key_enc, AES_MAX_KEYLENGTH);

	ctx->key_len = keylen;
	return 0;
}

static int aspeed_aes_ctr_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR);
}

static int aspeed_aes_ctr_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR);
}

static int aspeed_aes_ofb_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB);
}

static int aspeed_aes_ofb_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB);
}

static int aspeed_aes_cfb_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB);
}

static int aspeed_aes_cfb_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB);
}

static int aspeed_aes_ecb_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB);
}

static int aspeed_aes_ecb_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB);
}

static int aspeed_aes_cbc_decrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC);
}

static int aspeed_aes_cbc_encrypt(struct skcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC);
}

static int aspeed_crypto_cra_init(struct crypto_skcipher *tfm)
{
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct skcipher_alg *alg = crypto_skcipher_alg(tfm);
	struct aspeed_crypto_alg *crypto_alg;

	CIPHER_DBG("\n");
	crypto_alg = container_of(alg, struct aspeed_crypto_alg, alg.skcipher);

	ctx->crypto_dev = crypto_alg->crypto_dev;
	ctx->cipher_key = dma_alloc_coherent(ctx->crypto_dev->dev, PAGE_SIZE, &ctx->cipher_key_dma, GFP_KERNEL);
	ctx->start = aspeed_crypto_skcipher_trigger;
	return 0;
}

static void aspeed_crypto_cra_exit(struct crypto_skcipher *tfm)
{
	struct aspeed_cipher_ctx *ctx = crypto_skcipher_ctx(tfm);

	CIPHER_DBG("\n");
	//disable clk ??
	dma_free_coherent(ctx->crypto_dev->dev, PAGE_SIZE, ctx->cipher_key, ctx->cipher_key_dma);

	return;
}

static int aspeed_aead_complete(struct aspeed_crypto_dev *crypto_dev, int err)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct aead_request *req = aead_request_cast(sk_engine->areq);
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(crypto_aead_reqtfm(req));

	CIPHER_DBG("\n");
	memcpy(req->iv, ctx->cipher_key, 16);
	sk_engine->flags &= ~CRYPTO_FLAGS_BUSY;
	if (sk_engine->is_async)
		req->base.complete(&req->base, err);

	aspeed_crypto_sk_handle_queue(crypto_dev, NULL);

	return err;
}

static int aspeed_aead_transfer(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct aead_request *req = aead_request_cast(sk_engine->areq);
	struct crypto_aead *cipher = crypto_aead_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(cipher);

	CIPHER_DBG("\n");
	if (req->src == req->dst) {
		dma_unmap_sg(crypto_dev->dev, req->src, ctx->src_nents, DMA_BIDIRECTIONAL);
	} else {
		dma_unmap_sg(crypto_dev->dev, req->src, ctx->src_nents, DMA_TO_DEVICE);
		dma_unmap_sg(crypto_dev->dev, req->dst, ctx->dst_nents, DMA_FROM_DEVICE);
	}

	return aspeed_aead_complete(crypto_dev, 0);
}

static int aspeed_aead_start(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct aead_request *req = aead_request_cast(sk_engine->areq);
	struct crypto_aead *cipher = crypto_aead_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(cipher);
	struct aspeed_sg_list *src_list, *dst_list;
	dma_addr_t src_dma_addr, dst_dma_addr;
	struct scatterlist *s;
	int i;
	int counter;

	CIPHER_DBG("\n");

	ctx->enc_cmd |= HACE_CMD_DES_SG_CTRL | HACE_CMD_SRC_SG_CTRL | HACE_CMD_AES_KEY_HW_EXP;

	if (req->dst == req->src) {
		if (!dma_map_sg(crypto_dev->dev, req->src, ctx->src_nents, DMA_BIDIRECTIONAL)) {
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(src) error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	} else {
		if (!dma_map_sg(crypto_dev->dev, req->src, ctx->src_nents, DMA_TO_DEVICE)) {
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(src) error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
		if (!dma_map_sg(crypto_dev->dev, req->dst, ctx->dst_nents, DMA_FROM_DEVICE)) {
			dma_unmap_sg(crypto_dev->dev, req->dst, ctx->dst_nents, DMA_FROM_DEVICE);
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(dst) error\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	}

	src_list = (struct aspeed_sg_list *) sk_engine->cipher_addr;
	dst_list = (struct aspeed_sg_list *) sk_engine->dst_sg_addr;
	src_dma_addr = sk_engine->cipher_dma_addr;
	dst_dma_addr = sk_engine->dst_sg_dma_addr;
	if (req->dst == req->src) {
		for_each_sg(req->src, s, ctx->src_nents, i) {
			src_list[i].phy_addr = sg_dma_address(s);
			src_list[i].len = s->length;
			dst_list[i].phy_addr = sg_dma_address(s);
			dst_list[i].len = s->length;
		}
		dst_list[ctx->dst_nents - 1].len |= BIT(31); //TODO
		counter = 16;
		for (i = ctx->src_nents - 1; i >= 0; i--) {
			if (src_list[i].len > counter) {
				src_list[i].len -= counter;
				src_list[i].len |= BIT(31);
				break;
			} else {
				counter -= src_list[i].len;
				src_list[i].len = 0;
			}
		}
	} else {
		for_each_sg(req->src, s, ctx->src_nents, i) {
			src_list[i].phy_addr = sg_dma_address(s);
			src_list[i].len = s->length;
		}
		src_list[ctx->dst_nents - 1].len |= BIT(31);
		for_each_sg(req->dst, s, ctx->dst_nents, i) {
			dst_list[i].phy_addr = sg_dma_address(s);
			dst_list[i].len = s->length;
		}
		dst_list[ctx->dst_nents - 1].len |= BIT(31);
	}
	printk("src_list\n");
	for (i = 0; i < ctx->src_nents; i++) {
		printk("len: %x\n", src_list[i].len);
		printk("addr: %x\n", src_list[i].phy_addr);
	}
	printk("dst_list\n");
	for (i = 0; i < ctx->dst_nents; i++) {
		printk("len: %x\n", dst_list[i].len);
		printk("addr: %x\n", dst_list[i].phy_addr);
	}

#ifdef CONFIG_CRYPTO_DEV_ASPEED_SK_INT
	sk_engine->resume = aspeed_aead_transfer;
#endif
	aspeed_crypto_write(crypto_dev, src_dma_addr, ASPEED_HACE_SRC);
	aspeed_crypto_write(crypto_dev, dst_dma_addr, ASPEED_HACE_DEST);
	aspeed_crypto_write(crypto_dev, req->cryptlen, ASPEED_HACE_DATA_LEN);
	aspeed_crypto_write(crypto_dev, req->assoclen, ASPEED_HACE_TAG);
	aspeed_crypto_write(crypto_dev, ctx->enc_cmd, ASPEED_HACE_CMD);

	return aspeed_sk_wait_for_data_ready(crypto_dev, aspeed_aead_transfer);
}

int aspeed_crypto_aead_trigger(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_skcipher *sk_engine = &crypto_dev->sk_engine;
	struct aead_request *req = aead_request_cast(sk_engine->areq);
	struct crypto_aead *cipher = crypto_aead_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(cipher);

	CIPHER_DBG("\n");
	//for enable interrupt
#ifdef CONFIG_CRYPTO_DEV_ASPEED_SK_INT
	ctx->enc_cmd |= HACE_CMD_ISR_EN;
#endif
	aspeed_crypto_write(crypto_dev, ctx->cipher_key_dma, ASPEED_HACE_CONTEXT);
	ctx->dst_nents = sg_nents(req->dst);
	ctx->src_nents = sg_nents(req->src);

	printk("length: %d\n", req->src->length);
	printk("assoclen: %d\n", req->assoclen);
	printk("cryptlen: %d\n", req->cryptlen);

	return aspeed_aead_start(crypto_dev);
	// if ((ctx->dst_nents == 1) && (ctx->src_nents == 1))
	// 	return aspeed_sk_dma_start(crypto_dev);

	// return aspeed_sk_g6_start(crypto_dev);
}

static int aspeed_gcm_crypt(struct aead_request *req, u32 cmd)
{
	struct crypto_aead *tfm = crypto_aead_reqtfm(req);
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;
	u8 *test;
	int i;


	CIPHER_DBG("\n");
	memcpy(ctx->cipher_key, req->iv, 12);
	memset(ctx->cipher_key + 12, 0, 3);
	memset(ctx->cipher_key + 15, 1, 1);

	test = (u8 *)ctx->cipher_key;
	for (i = 0; i < ctx->key_len + 32; i++) {
		printk("%x ", test[i]);
	}
	printk("\n");

	ctx->enc_cmd = cmd;

	return aspeed_crypto_sk_handle_queue(crypto_dev, &req->base);
}

static void aspeed_gcm_subkey_done(struct crypto_async_request *req, int err)
{
	struct aspeed_gcm_subkey_result *res = req->data;

	CIPHER_DBG("\n");
	if (err == -EINPROGRESS)
		return;

	res->err = err;
	complete(&res->completion);
}

static int aspeed_gcm_subkey(struct crypto_skcipher *tfm, const u8 *data,
			     unsigned int len, const u8 *key, unsigned int keylen)
{
	struct aspeed_gcm_subkey_result result;
	struct skcipher_request *req;
	struct scatterlist sg[1];
	int ret;
	int i;
	CIPHER_DBG("\n");
	for (i = 0; i < len; i++) {
		printk("%2x ", data[i]);
	}
	printk("\n");

	init_completion(&result.completion);
	req = skcipher_request_alloc(tfm, GFP_KERNEL);

	skcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				      aspeed_gcm_subkey_done, &result);

	crypto_skcipher_clear_flags(tfm, ~0);
	crypto_skcipher_setkey(tfm, key, keylen);
	sg_init_one(sg, data, len);
	// sg_init_one(sg, enc_data, 16);
	skcipher_request_set_crypt(req, sg, sg, len, NULL);
	ret = crypto_skcipher_encrypt(req);

	switch (ret) {
	case 0:
		break;
	case -EINPROGRESS:
	case -EBUSY:
		wait_for_completion(&result.completion);
		ret = result.err;
		if (!ret)
			break;
	/* fall through */
	default:
		pr_err("ecb(aes) enc error");
		goto out;
	}
	for (i = 0; i < len; i++) {
		printk("%x ", data[i]);
	}
	printk("\n");
	// memcpy(data, enc_data, 16);
out:
	skcipher_request_free(req);
	return ret;
}

static int aspeed_gcm_setkey(struct crypto_aead *tfm, const u8 *key,
			     unsigned int keylen)
{
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(tfm);
	u8 data[16] = {0};
	// int i;

	CIPHER_DBG("bits : %d \n", (keylen * 8));

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256) {
		crypto_aead_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}
	// ctx->enc_cmd |= HACE_CMD_AES_KEY_HW_EXP;

	memcpy(ctx->cipher_key + 16, key, keylen);

	aspeed_gcm_subkey(ctx->aes, data, 16, key, keylen);
	memcpy(ctx->cipher_key + 16 + keylen, data, 16);

	ctx->key_len = keylen;
	return 0;
}

static int aspeed_gcm_setauthsize(struct crypto_aead *tfm,
				  unsigned int authsize)
{
	/* Same as gcm_authsize() from crypto/gcm.c */
	CIPHER_DBG("\n");
	switch (authsize) {
	case 4:
	case 8:
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int aspeed_gcm_encrypt(struct aead_request *req)
{
	return aspeed_gcm_crypt(req, HACE_CMD_GCM | HACE_CMD_ENCRYPT);
}

static int aspeed_gcm_decrypt(struct aead_request *req)
{
	return aspeed_gcm_crypt(req, HACE_CMD_GCM | HACE_CMD_DECRYPT);
}

static int aspeed_gcm_init(struct crypto_aead *tfm)
{
	struct aspeed_cipher_ctx *ctx = crypto_aead_ctx(tfm);
	struct aead_alg *alg = crypto_aead_alg(tfm);
	struct aspeed_crypto_alg *crypto_alg;

	CIPHER_DBG("\n");
	crypto_alg = container_of(alg, struct aspeed_crypto_alg, alg.aead);

	ctx->crypto_dev = crypto_alg->crypto_dev;
	ctx->cipher_key = dma_alloc_coherent(ctx->crypto_dev->dev, PAGE_SIZE, &ctx->cipher_key_dma, GFP_KERNEL);
	ctx->start = aspeed_crypto_aead_trigger;

	ctx->aes = crypto_alloc_skcipher("ecb(aes)", 0, CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(ctx->aes)) {
		pr_err("aspeed-gcm: base driver 'ecb(aes)' could not be loaded.\n");
		return PTR_ERR(ctx->aes);
	}
	return 0;
}

static void aspeed_gcm_exit(struct crypto_aead *tfm)
{

}

struct aspeed_crypto_alg aspeed_crypto_algs[] = {
	{
		.alg.skcipher = {
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= aspeed_aes_setkey,
			.encrypt	= aspeed_aes_ecb_encrypt,
			.decrypt	= aspeed_aes_ecb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ecb(aes)",
				.cra_driver_name	= "aspeed-ecb-aes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= AES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= AES_BLOCK_SIZE,
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= aspeed_aes_setkey,
			.encrypt	= aspeed_aes_cbc_encrypt,
			.decrypt	= aspeed_aes_cbc_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "cbc(aes)",
				.cra_driver_name	= "aspeed-cbc-aes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= AES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= AES_BLOCK_SIZE,
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= aspeed_aes_setkey,
			.encrypt	= aspeed_aes_cfb_encrypt,
			.decrypt	= aspeed_aes_cfb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "cfb(aes)",
				.cra_driver_name	= "aspeed-cfb-aes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= AES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= AES_BLOCK_SIZE,
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= aspeed_aes_setkey,
			.encrypt	= aspeed_aes_ofb_encrypt,
			.decrypt	= aspeed_aes_ofb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ofb(aes)",
				.cra_driver_name	= "aspeed-ofb-aes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= AES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_des_ecb_encrypt,
			.decrypt	= aspeed_des_ecb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ecb(des)",
				.cra_driver_name	= "aspeed-ecb-des",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_des_cbc_encrypt,
			.decrypt	= aspeed_des_cbc_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "cbc(des)",
				.cra_driver_name	= "aspeed-cbc-des",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_des_cfb_encrypt,
			.decrypt	= aspeed_des_cfb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "cfb(des)",
				.cra_driver_name	= "aspeed-cfb-des",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_des_ofb_encrypt,
			.decrypt	= aspeed_des_ofb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ofb(des)",
				.cra_driver_name	= "aspeed-ofb-des",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_ecb_encrypt,
			.decrypt	= aspeed_tdes_ecb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ecb(des3_ede)",
				.cra_driver_name	= "aspeed-ecb-tdes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_ecb_encrypt,
			.decrypt	= aspeed_tdes_ecb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ecb(des3_ede)",
				.cra_driver_name	= "aspeed-ecb-tdes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_cbc_encrypt,
			.decrypt	= aspeed_tdes_cbc_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "cbc(des3_ede)",
				.cra_driver_name	= "aspeed-cbc-tdes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_cfb_encrypt,
			.decrypt	= aspeed_tdes_cfb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "cfb(des3_ede)",
				.cra_driver_name	= "aspeed-cfb-tdes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_ofb_encrypt,
			.decrypt	= aspeed_tdes_ofb_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ofb(des3_ede)",
				.cra_driver_name	= "aspeed-ofb-tdes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.min_keysize	= 1,
			.max_keysize	= 256,
			.setkey		= aspeed_rc4_setkey,
			.encrypt	= aspeed_rc4_encrypt,
			.decrypt	= aspeed_rc4_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ecb(arc4)",
				.cra_driver_name	= "aspeed-arc4",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= 1,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	}
};

struct aspeed_crypto_alg aspeed_crypto_algs_g6[] = {
	{
		.alg.skcipher = {
			.ivsize		= AES_BLOCK_SIZE,
			.min_keysize	= AES_MIN_KEY_SIZE,
			.max_keysize	= AES_MAX_KEY_SIZE,
			.setkey		= aspeed_aes_setkey,
			.encrypt	= aspeed_aes_ctr_encrypt,
			.decrypt	= aspeed_aes_ctr_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ctr(aes)",
				.cra_driver_name	= "aspeed-ctr-aes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= AES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES_KEY_SIZE,
			.max_keysize	= DES_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_des_ctr_encrypt,
			.decrypt	= aspeed_des_ctr_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ctr(des)",
				.cra_driver_name	= "aspeed-ctr-des",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},
	{
		.alg.skcipher = {
			.ivsize		= DES_BLOCK_SIZE,
			.min_keysize	= DES3_EDE_KEY_SIZE,
			.max_keysize	= DES3_EDE_KEY_SIZE,
			.setkey		= aspeed_des_setkey,
			.encrypt	= aspeed_tdes_ctr_encrypt,
			.decrypt	= aspeed_tdes_ctr_decrypt,
			.init		= aspeed_crypto_cra_init,
			.exit		= aspeed_crypto_cra_exit,
			.base = {
				.cra_name		= "ctr(des3_ede)",
				.cra_driver_name	= "aspeed-ctr-tdes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= DES_BLOCK_SIZE,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			}
		}
	},

};
struct aspeed_crypto_alg aspeed_aead_algs_g6[] = {
	{
		.alg.aead = {
			.setkey		= aspeed_gcm_setkey,
			.setauthsize	= aspeed_gcm_setauthsize,
			.encrypt	= aspeed_gcm_encrypt,
			.decrypt	= aspeed_gcm_decrypt,
			.init		= aspeed_gcm_init,
			.exit		= aspeed_gcm_exit,
			.ivsize		= 12,
			.maxauthsize	= AES_BLOCK_SIZE,

			.base = {
				.cra_name		= "gcm(aes)",
				.cra_driver_name	= "aspeed-gcm-aes",
				.cra_priority		= 300,
				.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY | CRYPTO_ALG_ASYNC,
				.cra_blocksize		= 1,
				.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
				.cra_alignmask		= 0x0f,
				.cra_module		= THIS_MODULE,
			},
		}
	}
};

int aspeed_register_skcipher_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;
	// for (i = 0; i < ARRAY_SIZE(aspeed_crypto_algs); i++) {
	// 	aspeed_crypto_algs[i].crypto_dev = crypto_dev;
	// 	err = crypto_register_skcipher(&aspeed_crypto_algs[i].alg.skcipher);
	// 	if (err)
	// 		return err;
	// }
	if (crypto_dev->version == 6) {
		// for (i = 0; i < ARRAY_SIZE(aspeed_crypto_algs_g6); i++) {
		// 	aspeed_crypto_algs_g6[i].crypto_dev = crypto_dev;
		// 	err = crypto_register_skcipher(&aspeed_crypto_algs_g6[i].alg.skcipher);
		// 	if (err)
		// 		return err;
		// }
		for (i = 0; i < ARRAY_SIZE(aspeed_aead_algs_g6); i++) {
			aspeed_aead_algs_g6[i].crypto_dev = crypto_dev;
			err = crypto_register_aead(&aspeed_aead_algs_g6[i].alg.aead);
			if (err)
				return err;
		}
	}
	return 0;
}
