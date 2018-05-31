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

//#define ASPEED_CIPHER_DEBUG

#ifdef ASPEED_CIPHER_DEBUG
//#define CIPHER_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define CIPHER_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)

#else
#define CIPHER_DBG(fmt, args...)
#endif

int aspeed_crypto_ablkcipher_trigger(struct aspeed_crypto_dev *crypto_dev)
{
	struct crypto_ablkcipher *cipher = crypto_ablkcipher_reqtfm(crypto_dev->ablk_req);
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(cipher);
	struct ablkcipher_request	*req = crypto_dev->ablk_req;
	struct scatterlist	*in_sg = req->src, *out_sg = req->dst;
	int nbytes = 0;
//	struct scatterlist *sg;
//	int i = 0;

	if (ctx->enc_cmd & HACE_CMD_RC4) {
		if(!ctx->rc4_installed) {
			*(u32 *)(crypto_dev->ctx_buf + 8) = 0x0001;
			memcpy(crypto_dev->ctx_buf + 16, ctx->key.arc4, 256);
			ctx->rc4_installed = 1;
		};
	} else {
		if(ctx->enc_cmd & HACE_CMD_DES_SELECT) {
			if (ctx->iv) {
				memcpy(crypto_dev->ctx_buf + 8, ctx->iv, 8);
			}
			memcpy(crypto_dev->ctx_buf + 16, ctx->key.des, 24);
		} else {
			if (ctx->iv) {
				memcpy(crypto_dev->ctx_buf, ctx->iv, 16);
			}
			memcpy(crypto_dev->ctx_buf + 16, ctx->key.aes, 0xff);
		}
	}

#if 0
	for_each_sg(in_sg, sg, sg_nents(req->src), i) {
		printk("req->src %x [%d] : %x, len %d \n ",req->src, i, sg->dma_address, sg->length);
	}
	
	for_each_sg(out_sg, sg, sg_nents(req->dst), i) {
		printk("req->dst %x [%d] : %x, len %d \n ", req->dst, i, sg->dma_address, sg->length);
	}
#endif

#ifdef ASPEED_CRYPTO_IRQ
	crypto_dev->cmd |= HACE_CMD_ISR_EN;
#endif

	if((sg_nents(req->src) == 1) && (sg_nents(req->dst) == 1)) {
		//src dma map
		if (!dma_map_sg(crypto_dev->dev, req->src, 1, DMA_TO_DEVICE)) {
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(src)	error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		aspeed_crypto_write(crypto_dev, sg_dma_address(req->src), ASPEED_HACE_SRC);
		//dst
		if (!dma_map_sg(crypto_dev->dev, req->dst, 1, DMA_FROM_DEVICE)) {
			dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(dst)	error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		aspeed_crypto_write(crypto_dev, sg_dma_address(req->dst), ASPEED_HACE_DEST);

		aspeed_crypto_write(crypto_dev, req->nbytes, ASPEED_HACE_DATA_LEN);
		aspeed_crypto_write(crypto_dev, ctx->enc_cmd, ASPEED_HACE_CMD);
	
		while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_CRYPTO_BUSY);
		
	} else {
		nbytes = sg_copy_to_buffer(in_sg, sg_nents(req->src), crypto_dev->buf_in, req->nbytes);
		CIPHER_DBG("copy nbytes %d, req->nbytes %d , nb_in_sg %d, nb_out_sg %d \n", nbytes, req->nbytes, sg_nents(req->src), sg_nents(req->dst));
		if (!nbytes) {
			printk("nbytes error \n");
			return -EINVAL;
		}

		aspeed_crypto_write(crypto_dev, crypto_dev->dma_addr_in, ASPEED_HACE_SRC);
		aspeed_crypto_write(crypto_dev, crypto_dev->dma_addr_out, ASPEED_HACE_DEST);
		aspeed_crypto_write(crypto_dev, req->nbytes, ASPEED_HACE_DATA_LEN);
		aspeed_crypto_write(crypto_dev, ctx->enc_cmd, ASPEED_HACE_CMD);

		while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_CRYPTO_BUSY);
		
		nbytes = sg_copy_from_buffer(out_sg, sg_nents(req->dst), crypto_dev->buf_out, req->nbytes);
		CIPHER_DBG("sg_copy_from_buffer nbytes %d req->nbytes %d, cmd %x\n",nbytes, req->nbytes, ctx->enc_cmd);
		if (!nbytes) {
			printk("nbytes %d req->nbytes %d\n", nbytes, req->nbytes);
			return -EINVAL;
		}
		
	}

	return 0;
}

static int aspeed_rc4_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;

	CIPHER_DBG("\n");

	cmd |= HACE_CMD_RI_WO_DATA_ENABLE | 
			HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	ctx->enc_cmd = cmd;

	return aspeed_crypto_enqueue(crypto_dev, req);
}

static int aspeed_rc4_setkey(struct crypto_ablkcipher *cipher, const u8 *in_key,
			     unsigned int key_len)
{
	int i, j = 0, k = 0;

	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(cipher);

	CIPHER_DBG("keylen : %d : %s  \n", key_len, in_key);

	for (i = 0; i < 256; i++)
		ctx->key.arc4[i] = i;

	for (i = 0; i < 256; i++) {
		u32 a = ctx->key.arc4[i];
		j = (j + in_key[k] + a) & 0xff;
		ctx->key.arc4[i] = ctx->key.arc4[j];
		ctx->key.arc4[j] = a;
		if (++k >= key_len)
			k = 0;
	}

	ctx->key_len = 256;
	ctx->rc4_installed = 0;

	return 0;
}

static int aspeed_rc4_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_rc4_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_RC4);
}

static int aspeed_rc4_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_rc4_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_RC4);
}


static int aspeed_des_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;

	CIPHER_DBG("\n");

	cmd |= HACE_CMD_DES_SELECT | HACE_CMD_RI_WO_DATA_ENABLE |
	       HACE_CMD_CONTEXT_LOAD_ENABLE | HACE_CMD_CONTEXT_SAVE_ENABLE;

	ctx->iv = req->info;
	ctx->enc_cmd = cmd;

	return aspeed_crypto_enqueue(crypto_dev, req);
}

static int aspeed_des_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
			     unsigned int keylen)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(cipher);
	CIPHER_DBG("bits : %d : \n", keylen);

	if ((keylen != DES_KEY_SIZE) && (keylen != 2 * DES_KEY_SIZE) && (keylen != 3 * DES_KEY_SIZE)) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		printk("keylen fail %d bits \n", keylen);
		return -EINVAL;
	}

	memcpy(ctx->key.des, key, keylen);
	ctx->key_len = keylen;

	return 0;
}

static int aspeed_tdes_ctr_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ctr_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ofb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ofb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cfb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cfb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cbc_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_cbc_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ecb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_tdes_ecb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_TRIPLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ctr_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ctr_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ofb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ofb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cfb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cfb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cbc_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_cbc_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ecb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_des_ecb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_des_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB | HACE_CMD_SINGLE_DES | HACE_CMD_DES);
}

static int aspeed_aes_crypt(struct ablkcipher_request *req, u32 cmd)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(crypto_ablkcipher_reqtfm(req));
	struct aspeed_crypto_dev *crypto_dev = ctx->crypto_dev;

	ctx->iv = req->info;

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

	return aspeed_crypto_enqueue(crypto_dev, req);
}

static int aspeed_aes_setkey(struct crypto_ablkcipher *cipher, const u8 *key,
			     unsigned int keylen)
{
	struct aspeed_cipher_ctx *ctx = crypto_ablkcipher_ctx(cipher);
	struct crypto_aes_ctx gen_aes_key;

	CIPHER_DBG("bits : %d \n", (keylen * 8));

	if (keylen != AES_KEYSIZE_128 && keylen != AES_KEYSIZE_192 &&
	    keylen != AES_KEYSIZE_256) {
		crypto_ablkcipher_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	crypto_aes_expand_key(&gen_aes_key, key, keylen);
	memcpy(ctx->key.aes, gen_aes_key.key_enc, AES_MAX_KEYLENGTH);
	
	ctx->key_len = keylen;

	return 0;
}

static int aspeed_aes_ctr_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CTR);
}

static int aspeed_aes_ctr_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CTR);
}

static int aspeed_aes_ofb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_OFB);
}

static int aspeed_aes_ofb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_OFB);
}

static int aspeed_aes_cfb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CFB);
}

static int aspeed_aes_cfb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CFB);
}

static int aspeed_aes_ecb_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_ECB);
}

static int aspeed_aes_ecb_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_ECB);
}

static int aspeed_aes_cbc_decrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_DECRYPT | HACE_CMD_CBC);
}

static int aspeed_aes_cbc_encrypt(struct ablkcipher_request *req)
{
	CIPHER_DBG("\n");
	return aspeed_aes_crypt(req, HACE_CMD_ENCRYPT | HACE_CMD_CBC);
}

static int aspeed_crypto_cra_init(struct crypto_tfm *tfm)
{
	struct aspeed_cipher_ctx *ctx = crypto_tfm_ctx(tfm);
	struct crypto_alg *alg = tfm->__crt_alg;
	struct aspeed_crypto_alg *crypto_alg;
	crypto_alg = container_of(alg, struct aspeed_crypto_alg, alg.crypto);

	ctx->crypto_dev = crypto_alg->crypto_dev;
	ctx->iv = NULL;

	return 0;
}

static void aspeed_crypto_cra_exit(struct crypto_tfm *tfm)
{
	//disable clk ??
	return;
}

struct aspeed_crypto_alg aspeed_crypto_algs[] = {
	{
		.alg.crypto = {
			.cra_name 		= "ecb(aes)",
			.cra_driver_name 	= "aspeed-ecb-aes",
			.cra_priority 		= 300,
			.cra_flags 		= CRYPTO_ALG_TYPE_ABLKCIPHER |
						CRYPTO_ALG_ASYNC,
			.cra_blocksize 	= AES_BLOCK_SIZE,
			.cra_ctxsize 		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0x0f,
			.cra_type 		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init 		= aspeed_crypto_cra_init,
			.cra_exit 		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				//no iv
				.min_keysize 	= AES_MIN_KEY_SIZE,
				.max_keysize 	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt 	= aspeed_aes_ecb_encrypt,
				.decrypt	= aspeed_aes_ecb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name 		= "cbc(aes)",
			.cra_driver_name 	= "aspeed-cbc-aes",
			.cra_priority 		= 300,
			.cra_flags 		= CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
			.cra_blocksize 	= AES_BLOCK_SIZE,
			.cra_ctxsize 		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type 		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init 		= aspeed_crypto_cra_init,
			.cra_exit 		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize 	= AES_MIN_KEY_SIZE,
				.max_keysize 	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt 	= aspeed_aes_cbc_encrypt,
				.decrypt	= aspeed_aes_cbc_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "cfb(aes)",
			.cra_driver_name	= "aspeed-cfb-aes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize	= AES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt	= aspeed_aes_cfb_encrypt,
				.decrypt	= aspeed_aes_cfb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ofb(aes)",
			.cra_driver_name	= "aspeed-ofb-aes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize	= AES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt	= aspeed_aes_ofb_encrypt,
				.decrypt	= aspeed_aes_ofb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ctr(aes)",
			.cra_driver_name	= "aspeed-ctr-aes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize	= AES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= AES_BLOCK_SIZE,
				.min_keysize	= AES_MIN_KEY_SIZE,
				.max_keysize	= AES_MAX_KEY_SIZE,
				.setkey 		= aspeed_aes_setkey,
				.encrypt	= aspeed_aes_ctr_encrypt,
				.decrypt	= aspeed_aes_ctr_decrypt,
			},
		},
	},
#if 0
	{
		.alg.crypto = {
			.cra_name		= "ecb(des)",
			.cra_driver_name	= "aspeed-ecb-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				//no iv
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey		= aspeed_des_setkey,
				.encrypt	= aspeed_des_ecb_encrypt,
				.decrypt	= aspeed_des_ecb_decrypt,
			},
		},
	},
#endif	
	{
		.alg.crypto = {
			.cra_name		= "cbc(des)",
			.cra_driver_name	= "aspeed-cbc-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_des_cbc_encrypt,
				.decrypt	= aspeed_des_cbc_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "cfb(des)",
			.cra_driver_name	= "aspeed-cfb-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= DES_KEY_SIZE,
				.max_keysize	= DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_des_cfb_encrypt,
				.decrypt	= aspeed_des_cfb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ofb(des)",
			.cra_driver_name	= "aspeed-ofb-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_des_ofb_encrypt,
				.decrypt	= aspeed_des_ofb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ctr(des)",
			.cra_driver_name	= "aspeed-ctr-des",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
					CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_des_ctr_encrypt,
				.decrypt	= aspeed_des_ctr_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ecb(des3_ede)",
			.cra_driver_name	= "aspeed-ecb-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey		= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_ecb_encrypt,
				.decrypt	= aspeed_tdes_ecb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "cbc(des3_ede)",
			.cra_driver_name	= "aspeed-cbc-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_cbc_encrypt,
				.decrypt	= aspeed_tdes_cbc_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "cfb(des3_ede)",
			.cra_driver_name	= "aspeed-cfb-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_cfb_encrypt,
				.decrypt	= aspeed_tdes_cfb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ofb(des3_ede)",
			.cra_driver_name	= "aspeed-ofb-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_ofb_encrypt,
				.decrypt	= aspeed_tdes_ofb_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ctr(des3_ede)",
			.cra_driver_name	= "aspeed-ctr-tdes",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
			CRYPTO_ALG_ASYNC,
			.cra_blocksize		= DES_BLOCK_SIZE,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.ivsize		= DES_BLOCK_SIZE,
				.min_keysize	= 2 * DES_KEY_SIZE,
				.max_keysize	= 3 * DES_KEY_SIZE,
				.setkey 	= aspeed_des_setkey,
				.encrypt	= aspeed_tdes_ctr_encrypt,
				.decrypt	= aspeed_tdes_ctr_decrypt,
			},
		},
	},
	{
		.alg.crypto = {
			.cra_name		= "ecb(arc4)",
			.cra_driver_name	= "aspeed-ecb-arc4",
			.cra_priority		= 300,
			.cra_flags		= CRYPTO_ALG_TYPE_ABLKCIPHER |
									CRYPTO_ALG_ASYNC,
			.cra_blocksize		= 1,
			.cra_ctxsize		= sizeof(struct aspeed_cipher_ctx),
			.cra_alignmask	= 0xf,
			.cra_type		= &crypto_ablkcipher_type,
			.cra_module 		= THIS_MODULE,
			.cra_init		= aspeed_crypto_cra_init,
			.cra_exit		= aspeed_crypto_cra_exit,
			.cra_u.ablkcipher = {
				.min_keysize	= 1,
				.max_keysize	= 256,
				.setkey 	= aspeed_rc4_setkey,
				.encrypt	= aspeed_rc4_encrypt,
				.decrypt	= aspeed_rc4_decrypt,
			},
		},
	},
};

int aspeed_register_crypto_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;
	for (i = 0; i < ARRAY_SIZE(aspeed_crypto_algs); i++) {
		aspeed_crypto_algs[i].crypto_dev = crypto_dev;
		err = crypto_register_alg(&aspeed_crypto_algs[i].alg.crypto);
		if (err)
			return err;
	}
	return 0;
}
