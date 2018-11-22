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

#define ASPEED_AHASH_DEBUG

#ifdef ASPEED_AHASH_DEBUG
//#define AHASH_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define AHASH_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define AHASH_DBG(fmt, args...)
#endif

const u32 md5_iv[8] = {
	MD5_H0, MD5_H1, MD5_H2, MD5_H3,
	0, 0, 0, 0
};

const u32 sha1_iv[8] = {
	0x01234567UL, 0x89abcdefUL, 0xfedcba98UL, 0x76543210UL,
	0xf0e1d2c3UL, 0, 0, 0
};

const u32 sha224_iv[8] = {
	0xd89e05c1UL, 0x07d57c36UL, 0x17dd7030UL, 0x39590ef7UL,
	0x310bc0ffUL, 0x11155868UL, 0xa78ff964UL, 0xa44ffabeUL
};

const u32 sha256_iv[8] = {
	0x67e6096aUL, 0x85ae67bbUL, 0x72f36e3cUL, 0x3af54fa5UL,
	0x7f520e51UL, 0x8c68059bUL, 0xabd9831fUL, 0x19cde05bUL
};

int aspeed_crypto_ahash_trigger(struct aspeed_crypto_dev *crypto_dev,
				aspeed_crypto_fn_t resume)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	// int i;

	AHASH_DBG("\n");
#ifdef CONFIG_CRYPTO_DEV_ASPEED_AHASH_INT
	rctx->cmd |= HASH_CMD_INT_ENABLE;
	ahash_engine->resume = resume;
#endif

	// printk("padding buffer\n");
	// for (i = 0; i < rctx->bufcnt; i++) {
	// 	printk("%x ", rctx->buffer[i]);
	// }
	// printk("digest buffer\n");
	// for (i = 0; i < rctx->digsize; i++) {
	// 	printk("%x ", rctx->digest[i]);
	// }
	rctx->buffer_dma_addr = dma_map_single(crypto_dev->dev, rctx->buffer, rctx->bufcnt, DMA_TO_DEVICE);
	rctx->digest_dma_addr = dma_map_single(crypto_dev->dev, rctx->digest, rctx->digsize, DMA_TO_DEVICE);

	aspeed_crypto_write(crypto_dev, rctx->buffer_dma_addr, ASPEED_HACE_HASH_SRC);
	aspeed_crypto_write(crypto_dev, rctx->digest_dma_addr, ASPEED_HACE_HASH_DIGEST_BUFF);
	aspeed_crypto_write(crypto_dev, rctx->digest_dma_addr, ASPEED_HACE_HASH_KEY_BUFF);
	aspeed_crypto_write(crypto_dev, rctx->bufcnt, ASPEED_HACE_HASH_DATA_LEN);
	aspeed_crypto_write(crypto_dev, rctx->cmd, ASPEED_HACE_HASH_CMD);
	rctx->bufcnt = 0;
#ifndef CONFIG_CRYPTO_DEV_ASPEED_AHASH_INT
	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);
	aspeed_crypto_write(crypto_dev, sts, ASPEED_HACE_STS);
	resume(crypto_dev);
	return 0;
#endif
	return -EINPROGRESS;
}

static int aspeed_sham_append_sg(struct aspeed_sham_reqctx *rctx)
{
	size_t count;

	AHASH_DBG("\n");
	while ((rctx->bufcnt < rctx->buflen) && rctx->total) {
		count = min(rctx->src_sg->length - rctx->offset, rctx->total);
		count = min(count, rctx->buflen - rctx->bufcnt);
		printk("count:%d,bufcnt:%d,buflen:%d,total:%d\n", count, rctx->bufcnt, rctx->buflen, rctx->total);
		if (count <= 0) {
			/*
			* Check if count <= 0 because the buffer is full or
			* because the sg length is 0. In the latest case,
			* check if there is another sg in the list, a 0 length
			* sg doesn't necessarily mean the end of the sg list.
			*/
			if ((rctx->src_sg->length == 0) && !sg_is_last(rctx->src_sg)) {
				rctx->src_sg = sg_next(rctx->src_sg);
				continue;
			} else {
				break;
			}
		}

		scatterwalk_map_and_copy(rctx->buffer + rctx->bufcnt, rctx->src_sg,
					 rctx->offset, count, 0);

		rctx->bufcnt += count;
		rctx->offset += count;
		rctx->total -= count;

		if (rctx->offset == rctx->src_sg->length) {
			rctx->src_sg = sg_next(rctx->src_sg);
			if (rctx->src_sg)
				rctx->offset = 0;
			else
				rctx->total = 0;
		}
	}

	return 0;
}

static void aspeed_crypto_ahash_fill_padding(struct aspeed_sham_reqctx *rctx)
{
	unsigned int index, padlen;
	u64 bits[2];

	AHASH_DBG("\n");

	if (rctx->flags & SHA_FLAGS_MD5) {
		bits[0] = cpu_to_le64(rctx->digcnt[0] << 3);
		index = rctx->bufcnt & 0x3f;
		padlen = (index < 56) ? (56 - index) : ((64 + 56) - index);
		*(rctx->buffer + rctx->bufcnt) = 0x80;
		memset(rctx->buffer + rctx->bufcnt + 1, 0, padlen - 1);
		memcpy(rctx->buffer + rctx->bufcnt + padlen, bits, 8);
		rctx->bufcnt += padlen + 8;
	} else if (rctx->flags & (SHA_FLAGS_SHA1 | SHA_FLAGS_SHA224 | SHA_FLAGS_SHA256)) {
		bits[0] = cpu_to_be64(rctx->digcnt[0] << 3);
		index = rctx->bufcnt & 0x3f;
		padlen = (index < 56) ? (56 - index) : ((64 + 56) - index);
		*(rctx->buffer + rctx->bufcnt) = 0x80;
		memset(rctx->buffer + rctx->bufcnt + 1, 0, padlen - 1);
		memcpy(rctx->buffer + rctx->bufcnt + padlen, bits, 8);
		rctx->bufcnt += padlen + 8;
	} else {
		bits[1] = cpu_to_be64(rctx->digcnt[0] << 3);
		bits[0] = cpu_to_be64(rctx->digcnt[1] << 3 | rctx->digcnt[0] >> 61);
		index = rctx->bufcnt & 0x7f;
		padlen = (index < 112) ? (112 - index) : ((128 + 112) - index);
		*(rctx->buffer + rctx->bufcnt) = 0x80;
		memset(rctx->buffer + rctx->bufcnt + 1, 0, padlen - 1);
		memcpy(rctx->buffer + rctx->bufcnt + padlen, bits, 16);
		rctx->bufcnt += padlen + 16;
	}
}

static int aspeed_ahash_complete(struct aspeed_crypto_dev *crypto_dev, int err)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;

	AHASH_DBG("\n");

	ahash_engine->flags &= ~CRYPTO_FLAGS_BUSY;

	if (req->base.complete)
		req->base.complete(&req->base, err);

	aspeed_crypto_ahash_handle_queue(crypto_dev, NULL);
	return err;
}

static int aspeed_ahash_transfer(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");
	// dma_unmap_single(crypto_dev->dev, rctx->buffer_dma_addr, rctx->digsize, DMA_FROM_DEVICE);
	dma_unmap_single(crypto_dev->dev, rctx->digest_dma_addr, rctx->digsize, DMA_FROM_DEVICE);
	memcpy(req->result, rctx->digest, rctx->digsize);

	return aspeed_ahash_complete(crypto_dev, 0);
}

static int aspeed_ahash_update_resume(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");

	if ((rctx->bufcnt + rctx->total < rctx->buflen) || rctx->total == 0) {
		aspeed_sham_append_sg(rctx);
		if (rctx->flags & SHA_FLAGS_FINUP) {
			/* no final() after finup() */
			aspeed_crypto_ahash_fill_padding(rctx);
			return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_transfer);
		}
		dma_unmap_single(crypto_dev->dev, rctx->digest_dma_addr, rctx->digsize, DMA_FROM_DEVICE);
		aspeed_ahash_complete(crypto_dev, 0);
		return 0;
	}
	aspeed_sham_append_sg(rctx);
	return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_update_resume);
}

int aspeed_crypto_ahash_handle_queue(struct aspeed_crypto_dev *crypto_dev,
				     struct crypto_async_request *new_areq)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct crypto_async_request *areq, *backlog;
	struct aspeed_sham_reqctx *rctx;
	unsigned long flags;
	int ret = 0;

	AHASH_DBG("\n");
	spin_lock_irqsave(&ahash_engine->lock, flags);
	if (new_areq)
		ret = crypto_enqueue_request(&ahash_engine->queue, new_areq);
	if (ahash_engine->flags & CRYPTO_FLAGS_BUSY) {
		spin_unlock_irqrestore(&ahash_engine->lock, flags);
		return ret;
	}
	backlog = crypto_get_backlog(&ahash_engine->queue);
	areq = crypto_dequeue_request(&ahash_engine->queue);
	if (areq)
		ahash_engine->flags |= CRYPTO_FLAGS_BUSY;
	spin_unlock_irqrestore(&ahash_engine->lock, flags);

	if (!areq)
		return ret;

	if (backlog)
		backlog->complete(backlog, -EINPROGRESS);


	ahash_engine->ahash_req = ahash_request_cast(areq);
	rctx = ahash_request_ctx(ahash_engine->ahash_req);

	if (rctx->op == SHA_OP_UPDATE) {
		aspeed_sham_append_sg(rctx);
		aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_update_resume);
	} else if (rctx->op == SHA_OP_FINAL) {
		aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_transfer);
	}

	return ret;
}

static int aspeed_sham_update(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	struct aspeed_crypto_dev *crypto_dev = tctx->crypto_dev;

	AHASH_DBG("\n");

	rctx->total = req->nbytes;
	rctx->src_sg = req->src;
	rctx->offset = 0;
	rctx->op = SHA_OP_UPDATE;

	rctx->digcnt[0] += rctx->total;
	if (rctx->digcnt[0] < rctx->total)
		rctx->digcnt[1]++;

	if (rctx->bufcnt + rctx->total < rctx->buflen) {
		aspeed_sham_append_sg(rctx);
		return 0;
	}
	return aspeed_crypto_ahash_handle_queue(crypto_dev, &req->base);
}

static int aspeed_sham_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = tctx->crypto_dev;

	AHASH_DBG("req->nbytes %d , rctx->total %d\n", req->nbytes, rctx->total);
	rctx->op = SHA_OP_FINAL;
	aspeed_crypto_ahash_fill_padding(rctx);

	return aspeed_crypto_ahash_handle_queue(crypto_dev, &req->base);
}

static int aspeed_sham_finup(struct ahash_request *req)
{
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	int err1, err2;

	AHASH_DBG("\n");
	rctx->flags |= SHA_FLAGS_FINUP;

	err1 = aspeed_sham_update(req);
	if (err1 == -EINPROGRESS || err1 == -EBUSY)
		return err1;

	/*
	 * final() has to be always called to cleanup resources
	 * even if udpate() failed, except EINPROGRESS
	 */
	err2 = aspeed_sham_final(req);

	return err1 ? : err2;
}

static int aspeed_sham_init(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("digest size: %d\n", crypto_ahash_digestsize(tfm));

	rctx->cmd = HASH_CMD_ACC_MODE;
	rctx->flags = 0;

	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
		rctx->flags |= SHA_FLAGS_MD5;
		rctx->digsize = MD5_DIGEST_SIZE;
		rctx->block_size = MD5_HMAC_BLOCK_SIZE;
		memcpy(rctx->digest, md5_iv, 32);
		break;
	case SHA1_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		rctx->flags |= SHA_FLAGS_SHA1;
		rctx->digsize = SHA1_DIGEST_SIZE;
		rctx->block_size = SHA1_BLOCK_SIZE;
		memcpy(rctx->digest, sha1_iv, 32);
		break;
	case SHA224_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
		rctx->flags |= SHA_FLAGS_SHA224;
		rctx->digsize = SHA224_DIGEST_SIZE;
		rctx->block_size = SHA224_BLOCK_SIZE;
		memcpy(rctx->digest, sha224_iv, 32);
		break;
	case SHA256_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
		rctx->flags |= SHA_FLAGS_SHA256;
		rctx->digsize = SHA256_DIGEST_SIZE;
		rctx->block_size = SHA256_BLOCK_SIZE;
		memcpy(rctx->digest, sha256_iv, 32);
		break;
	default:
		printk("%d not support \n", crypto_ahash_digestsize(tfm));
		return -EINVAL;
		break;
	}
	rctx->bufcnt = 0;
	rctx->total = 0;
	rctx->digcnt[0] = 0;
	rctx->digcnt[1] = 0;
	rctx->buflen = SHA_BUFFER_LEN;

	//hmac cmd
	if (tctx->flags)
		rctx->cmd |= HASH_CMD_HMAC;

	return 0;
}

static int aspeed_sham_digest(struct ahash_request *req)
{
	AHASH_DBG("\n");
	return aspeed_sham_init(req) ? : aspeed_sham_finup(req);
}

static int aspeed_sham_setkey(struct crypto_ahash *tfm, const u8 *key,
			      unsigned int keylen)
{
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = tctx->crypto_dev;
	int err = 0;
	size_t			digsize;
	u32 cmd;

	AHASH_DBG("keylen %d crypto_dev %x \n", keylen, (u32)crypto_dev);

	switch (crypto_ahash_digestsize(tfm)) {
	case MD5_DIGEST_SIZE:
		cmd = HASH_CMD_MD5 | HASH_CMD_MD5_SWAP;
		digsize = MD5_DIGEST_SIZE;
		break;
	case SHA1_DIGEST_SIZE:
		cmd = HASH_CMD_SHA1 | HASH_CMD_SHA_SWAP;
		digsize = SHA1_DIGEST_SIZE;
		break;
	case SHA224_DIGEST_SIZE:
		cmd = HASH_CMD_SHA224 | HASH_CMD_SHA_SWAP;
		digsize = SHA224_DIGEST_SIZE;
		break;
	case SHA256_DIGEST_SIZE:
		cmd = HASH_CMD_SHA256 | HASH_CMD_SHA_SWAP;
		digsize = SHA256_DIGEST_SIZE;
		break;
	default:
		printk("%d not support \n", crypto_ahash_digestsize(tfm));
		return -EINVAL;
		break;
	}

#if 0
	printk("key : \n");
	print_hex_dump(KERN_CONT, "", DUMP_PREFIX_OFFSET,
		       16, 1,
		       key, keylen, false);
	printk("\n");
#endif
	// memset(tctx->hash_digst, 0, 64);

	// if (keylen > 64) {
	// 	AHASH_DBG("gen hash key keylen %d crypto_dev->hash_key_dma trigger  %x \n", keylen, tctx->hash_digst_dma);
	// 	//gen hash(key)
	// 	memcpy(crypto_dev->hash_src, key, keylen);
	// 	aspeed_crypto_write(crypto_dev, crypto_dev->hash_src_dma, ASPEED_HACE_HASH_SRC);
	// 	aspeed_crypto_write(crypto_dev, tctx->hash_digst_dma, ASPEED_HACE_HASH_DIGEST_BUFF);
	// 	aspeed_crypto_write(crypto_dev, keylen, ASPEED_HACE_HASH_DATA_LEN);
	// 	aspeed_crypto_write(crypto_dev, cmd, ASPEED_HACE_HASH_CMD);
	// 	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);
	// 	//for workaround for SHA224 fill 32 bytes
	// 	if (digsize == SHA224_DIGEST_SIZE) {
	// 		memset(tctx->hash_digst + 28, 0, 4);
	// 	}
	// } else {
	// 	memcpy(tctx->hash_digst, key, keylen);
	// }

	// cmd |= HASH_CMD_HMAC;

	// aspeed_crypto_write(crypto_dev, tctx->hash_digst_dma, ASPEED_HACE_HASH_SRC);
	// aspeed_crypto_write(crypto_dev, tctx->hmac_key_dma, ASPEED_HACE_HASH_KEY_BUFF);
	// aspeed_crypto_write(crypto_dev, 0x40, ASPEED_HACE_HASH_DATA_LEN);
	// aspeed_crypto_write(crypto_dev, cmd | BIT(8), ASPEED_HACE_HASH_CMD);
	// while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);

	return err;
}

static int aspeed_sham_cra_init_alg(struct crypto_tfm *tfm, const char *alg_base)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);
	struct aspeed_crypto_alg *algt;
	struct ahash_alg *alg = __crypto_ahash_alg(tfm->__crt_alg);

	algt = container_of(alg, struct aspeed_crypto_alg, alg.ahash);
	tctx->crypto_dev = algt->crypto_dev;

	AHASH_DBG("%s crypto dev %x \n", crypto_tfm_alg_name(tfm), (u32)tctx->crypto_dev);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm), sizeof(struct aspeed_sham_reqctx));

	return 0;
}

static int aspeed_sham_cra_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, NULL);
}

static int aspeed_sham_cra_sha1_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha1");
}

static int aspeed_sham_cra_sha224_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha224");
}

static int aspeed_sham_cra_sha256_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha256");
}

static int aspeed_sham_cra_md5_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "md5");
}

// static int aspeed_sha_cra_sha384_init(struct crypto_tfm *tfm)
// {
// 	return aspeed_sham_cra_init_alg(tfm, "sha384");
// }

// static int aspeed_sha_cra_sha512_init(struct crypto_tfm *tfm)
// {
// 	return aspeed_sham_cra_init_alg(tfm, "sha512");
// }

static void aspeed_sham_cra_exit(struct crypto_tfm *tfm)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);

	AHASH_DBG("\n");

	crypto_free_shash(tctx->fallback);

	if (tctx->flags) {
		struct aspeed_sha_hmac_ctx *bctx = tctx->base;
		AHASH_DBG("HMAC \n");
		crypto_free_shash(bctx->shash);
	}
}

static int aspeed_sham_export(struct ahash_request *req, void *out)
{
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	AHASH_DBG("rctx->bufcnt %d \n", rctx->bufcnt);

	memcpy(out, rctx, sizeof(*rctx));
	return 0;
}

static int aspeed_sham_import(struct ahash_request *req, const void *in)
{
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	const struct aspeed_sham_reqctx *ctx_in = in;
	AHASH_DBG("ctx_in->bufcnt %d \n", ctx_in->bufcnt);

	memcpy(rctx, in, sizeof(*rctx));
	return 0;
}

struct aspeed_crypto_alg aspeed_ahash_algs[] = {
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = MD5_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "md5",
					.cra_driver_name	= "aspeed-md5",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA1_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "sha1",
					.cra_driver_name	= "aspeed-sha1",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA1_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA256_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "sha256",
					.cra_driver_name	= "aspeed-sha256",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA256_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA224_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "sha224",
					.cra_driver_name	= "aspeed-sha224",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA224_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
	// {
	// 	.alg.ahash = {
	// 		.init	= aspeed_sham_init,
	// 		.update	= aspeed_sham_update,
	// 		.final	= aspeed_sham_final,
	// 		.finup	= aspeed_sham_finup,
	// 		.digest	= aspeed_sham_digest,
	// 		.setkey	= aspeed_sham_setkey,
	// 		.export	= aspeed_sham_export,
	// 		.import	= aspeed_sham_import,
	// 		.halg = {
	// 			.digestsize = MD5_DIGEST_SIZE,
	// 			.statesize = sizeof(struct aspeed_sham_reqctx),
	// 			.base = {
	// 				.cra_name		= "hmac(md5)",
	// 				.cra_driver_name	= "aspeed-hmac-md5",
	// 				.cra_priority		= 300,
	// 				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
	// 				CRYPTO_ALG_ASYNC |
	// 				CRYPTO_ALG_KERN_DRIVER_ONLY,
	// 				.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
	// 				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
	// 				.cra_alignmask		= 0,
	// 				.cra_module		= THIS_MODULE,
	// 				.cra_init		= aspeed_sham_cra_md5_init,
	// 				.cra_exit		= aspeed_sham_cra_exit,
	// 			}
	// 		}
	// 	},
	// },

	// {
	// 	.alg.ahash = {
	// 		.init	= aspeed_sham_init,
	// 		.update	= aspeed_sham_update,
	// 		.final	= aspeed_sham_final,
	// 		.finup	= aspeed_sham_finup,
	// 		.digest	= aspeed_sham_digest,
	// 		.setkey	= aspeed_sham_setkey,
	// 		.export	= aspeed_sham_export,
	// 		.import	= aspeed_sham_import,
	// 		.halg = {
	// 			.digestsize = SHA1_DIGEST_SIZE,
	// 			.statesize = sizeof(struct aspeed_sham_reqctx),
	// 			.base = {
	// 				.cra_name		= "hmac(sha1)",
	// 				.cra_driver_name	= "aspeed-hmac-sha1",
	// 				.cra_priority		= 300,
	// 				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
	// 				CRYPTO_ALG_ASYNC |
	// 				CRYPTO_ALG_KERN_DRIVER_ONLY,
	// 				.cra_blocksize		= SHA1_BLOCK_SIZE,
	// 				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
	// 				.cra_alignmask		= 0,
	// 				.cra_module		= THIS_MODULE,
	// 				.cra_init		= aspeed_sham_cra_sha1_init,
	// 				.cra_exit		= aspeed_sham_cra_exit,
	// 			}
	// 		}
	// 	},
	// },
	// {
	// 	.alg.ahash = {
	// 		.init	= aspeed_sham_init,
	// 		.update	= aspeed_sham_update,
	// 		.final	= aspeed_sham_final,
	// 		.finup	= aspeed_sham_finup,
	// 		.digest	= aspeed_sham_digest,
	// 		.setkey	= aspeed_sham_setkey,
	// 		.export	= aspeed_sham_export,
	// 		.import	= aspeed_sham_import,
	// 		.halg = {
	// 			.digestsize = SHA224_DIGEST_SIZE,
	// 			.statesize = sizeof(struct aspeed_sham_reqctx),
	// 			.base = {
	// 				.cra_name		= "hmac(sha224)",
	// 				.cra_driver_name	= "aspeed-hmac-sha224",
	// 				.cra_priority		= 300,
	// 				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
	// 				CRYPTO_ALG_ASYNC |
	// 				CRYPTO_ALG_KERN_DRIVER_ONLY,
	// 				.cra_blocksize		= SHA224_BLOCK_SIZE,
	// 				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
	// 				.cra_alignmask		= 0,
	// 				.cra_module		= THIS_MODULE,
	// 				.cra_init		= aspeed_sham_cra_sha224_init,
	// 				.cra_exit		= aspeed_sham_cra_exit,
	// 			}
	// 		}
	// 	},
	// },
	// {
	// 	.alg.ahash = {
	// 		.init	= aspeed_sham_init,
	// 		.update	= aspeed_sham_update,
	// 		.final	= aspeed_sham_final,
	// 		.finup	= aspeed_sham_finup,
	// 		.digest	= aspeed_sham_digest,
	// 		.setkey	= aspeed_sham_setkey,
	// 		.export	= aspeed_sham_export,
	// 		.import	= aspeed_sham_import,
	// 		.halg = {
	// 			.digestsize = SHA256_DIGEST_SIZE,
	// 			.statesize = sizeof(struct aspeed_sham_reqctx),
	// 			.base = {
	// 				.cra_name		= "hmac(sha256)",
	// 				.cra_driver_name	= "aspeed-hmac-sha256",
	// 				.cra_priority		= 300,
	// 				.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
	// 				CRYPTO_ALG_ASYNC |
	// 				CRYPTO_ALG_KERN_DRIVER_ONLY,
	// 				.cra_blocksize		= SHA256_BLOCK_SIZE,
	// 				.cra_ctxsize		= sizeof(struct aspeed_sham_ctx),
	// 				.cra_alignmask		= 0,
	// 				.cra_module		= THIS_MODULE,
	// 				.cra_init		= aspeed_sham_cra_sha256_init,
	// 				.cra_exit		= aspeed_sham_cra_exit,
	// 			}
	// 		}
	// 	},
	// },
};

int aspeed_register_ahash_algs(struct aspeed_crypto_dev *crypto_dev)
{
	int i;
	int err = 0;

	for (i = 0; i < ARRAY_SIZE(aspeed_ahash_algs); i++) {
		aspeed_ahash_algs[i].crypto_dev = crypto_dev;
		err = crypto_register_ahash(&aspeed_ahash_algs[i].alg.ahash);
		if (err)
			return err;
	}
	return 0;
}
