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

// #define ASPEED_AHASH_DEBUG

#ifdef ASPEED_AHASH_DEBUG
//#define AHASH_DBG(fmt, args...) printk(KERN_DEBUG "%s() " fmt, __FUNCTION__, ## args)
#define AHASH_DBG(fmt, args...) printk("%s() " fmt, __FUNCTION__, ## args)
#else
#define AHASH_DBG(fmt, args...)
#endif

static const u32 md5_iv[8] = {
	MD5_H0, MD5_H1, MD5_H2, MD5_H3,
	0, 0, 0, 0
};

static const u32 sha1_iv[8] = {
	0x01234567UL, 0x89abcdefUL, 0xfedcba98UL, 0x76543210UL,
	0xf0e1d2c3UL, 0, 0, 0
};

static const u32 sha224_iv[8] = {
	0xd89e05c1UL, 0x07d57c36UL, 0x17dd7030UL, 0x39590ef7UL,
	0x310bc0ffUL, 0x11155868UL, 0xa78ff964UL, 0xa44ffabeUL
};

static const u32 sha256_iv[8] = {
	0x67e6096aUL, 0x85ae67bbUL, 0x72f36e3cUL, 0x3af54fa5UL,
	0x7f520e51UL, 0x8c68059bUL, 0xabd9831fUL, 0x19cde05bUL
};

static const u32 sha384_iv[16] = {
	0x5d9dbbcbUL, 0xd89e05c1UL, 0x2a299a62UL, 0x07d57c36UL,
	0x5a015991UL, 0x17dd7030UL, 0xd8ec2f15UL, 0x39590ef7UL,
	0x67263367UL, 0x310bc0ffUL, 0x874ab48eUL, 0x11155868UL,
	0x0d2e0cdbUL, 0xa78ff964UL, 0x1d48b547UL, 0xa44ffabeUL
};

static const u32 sha512_iv[16] = {
	0x67e6096aUL, 0x08c9bcf3UL, 0x85ae67bbUL, 0x3ba7ca84UL,
	0x72f36e3cUL, 0x2bf894feUL, 0x3af54fa5UL, 0xf1361d5fUL,
	0x7f520e51UL, 0xd182e6adUL, 0x8c68059bUL, 0x1f6c3e2bUL,
	0xabd9831fUL, 0x6bbd41fbUL, 0x19cde05bUL, 0x79217e13UL
};

static void aspeed_crypto_ahash_iV(struct aspeed_sham_reqctx *rctx)
{
	if (rctx->flags & SHA_FLAGS_MD5)
		memcpy(rctx->digest, md5_iv, 32);
	else if (rctx->flags & SHA_FLAGS_SHA1)
		memcpy(rctx->digest, sha1_iv, 32);
	else if (rctx->flags & SHA_FLAGS_SHA224)
		memcpy(rctx->digest, sha224_iv, 32);
	else if (rctx->flags & SHA_FLAGS_SHA256)
		memcpy(rctx->digest, sha256_iv, 32);
	else if (rctx->flags & SHA_FLAGS_SHA384)
		memcpy(rctx->digest, sha384_iv, 64);
	else if (rctx->flags & SHA_FLAGS_SHA512)
		memcpy(rctx->digest, sha512_iv, 64);
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

static int aspeed_ahash_dma_prepare(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	int length;
	int remain;
	// int i;
	// u8 *tmp;

	AHASH_DBG("\n");
	length = rctx->total + rctx->bufcnt;
	remain = length % rctx->block_size;

	// printk("length: %d, remain: %d\n", length, remain);
	if (rctx->bufcnt)
		memcpy(ahash_engine->ahash_src_addr, rctx->buffer, rctx->bufcnt);
	if (rctx-> total + rctx->bufcnt < 0xa000) {
		scatterwalk_map_and_copy(ahash_engine->ahash_src_addr + rctx->bufcnt,
					 rctx->src_sg, rctx->offset, rctx->total - remain, 0);
		rctx->offset += rctx->total - remain;
	} else {
		printk("too long!!!!!!!!!!!!!!!");
	}

	scatterwalk_map_and_copy(rctx->buffer, rctx->src_sg,
				 rctx->offset, remain, 0);
	rctx->bufcnt = remain;
	rctx->digest_dma_addr = dma_map_single(crypto_dev->dev, rctx->digest,
					       SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
	ahash_engine->src_length = length - remain;
	ahash_engine->src_dma = ahash_engine->ahash_src_dma_addr;
	ahash_engine->digeset_dma = rctx->digest_dma_addr;
	return 0;
}

static int aspeed_ahash_append_sg_map(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	struct aspeed_sg_list *src_list;
	struct scatterlist *s;
	int length;
	int remaining;
	int buf_sg = 0;
	int counter = 0;
	int ret;
	int i;
	char *buf;
	buf = sg_virt(rctx->src_sg);


	AHASH_DBG("\n");
	length = rctx->total + rctx->bufcnt;
	remaining = length % rctx->block_size;
	ret = dma_map_sg(crypto_dev->dev, rctx->src_sg, rctx->src_nents, DMA_TO_DEVICE);
	if (!ret) {
		dev_err(crypto_dev->dev, "[%s:%d] dma_map_sg(src) error\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	printk("sg_map: %d,nents: %d\n", ret, rctx->src_nents);
	src_list = (struct aspeed_sg_list *) ahash_engine->ahash_src_addr;
	printk("sg_map digsize\n");
	for (i = 0; i < 32; i++) {
		printk(KERN_CONT "%x ", rctx->digest[i]);
	}
	printk("\n");
	rctx->digest_dma_addr = dma_map_single(crypto_dev->dev, rctx->digest,
					       SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
	if (rctx->bufcnt != 0) {
		rctx->buffer_dma_addr = dma_map_single(crypto_dev->dev, rctx->buffer,
						       rctx->buflen + rctx->block_size, DMA_TO_DEVICE);
		src_list[0].phy_addr = rctx->buffer_dma_addr;
		src_list[0].len = rctx->bufcnt;
		buf_sg = 1;
	} else {
		buf_sg = 0;
	}
	for_each_sg(rctx->src_sg, s, rctx->src_nents, i) {
		src_list[i + buf_sg].phy_addr = sg_dma_address(s);
		src_list[i + buf_sg].len = s->length;
	}
	counter = 0;
	for (i = rctx->src_nents + buf_sg - 1; i >= 0; i--) {
		if (src_list[i].len + counter > remaining) {
			src_list[i].len -= remaining - counter;
			src_list[i].len |= BIT(31);
			break;
		} else {
			counter += src_list[i].len;
		}
	}
	// printk("sg_list\n");
	// for (i = 0; i < rctx->src_nents + buf_sg; i++) {
	// 	printk("addr: %x, len: %x\n", src_list[i].phy_addr, src_list[i].len);
	// }

	// printk("\n");
	// printk("src_sg\n");
	// for (i = 0; i < rctx->src_sg->length; i++) {
	// 	printk(KERN_CONT "%x ", buf[i]);
	// }
	// printk("\n");
	rctx->offset = rctx->total - remaining;
	// printk("rctx->offset: %d, rctx->total: %d, rctx->bufcnt: %d\n", rctx->offset, rctx->total, rctx->bufcnt);
	ahash_engine->src_length = length - remaining;
	ahash_engine->src_dma = ahash_engine->ahash_src_dma_addr;
	ahash_engine->digeset_dma = rctx->digest_dma_addr;
	return 0;
}

static int aspeed_ahash_complete(struct aspeed_crypto_dev *crypto_dev, int err)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;

	AHASH_DBG("\n");

	ahash_engine->flags &= ~CRYPTO_FLAGS_BUSY;

	if (req->base.complete)
		req->base.complete(&req->base, err);

	// tasklet_schedule(&ahash_engine->queue_task);
	aspeed_crypto_ahash_handle_queue(crypto_dev, NULL);
	return err;
}

static int aspeed_ahash_transfer(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");
	dma_unmap_single(crypto_dev->dev, rctx->digest_dma_addr,
			 SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
	dma_unmap_single(crypto_dev->dev, rctx->buffer_dma_addr,
			 rctx->buflen + rctx->block_size, DMA_TO_DEVICE);
	memcpy(req->result, rctx->digest, rctx->digsize);

	return aspeed_ahash_complete(crypto_dev, 0);
}

static int aspeed_ahash_hmac_resume(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_sha_hmac_ctx *bctx = tctx->base;

	AHASH_DBG("\n");
	dma_unmap_single(crypto_dev->dev, rctx->digest_dma_addr,
			 SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
	dma_unmap_single(crypto_dev->dev, rctx->buffer_dma_addr,
			 rctx->buflen + rctx->block_size, DMA_TO_DEVICE);
	memcpy(rctx->buffer, bctx->opad, rctx->block_size);
	memcpy(rctx->buffer + rctx->block_size, rctx->digest, rctx->digsize);
	rctx->bufcnt = rctx->block_size + rctx->digsize;
	rctx->digcnt[0] = rctx->block_size + rctx->digsize;

	aspeed_crypto_ahash_fill_padding(rctx);
	aspeed_crypto_ahash_iV(rctx);

	// memcpy(rctx->digest, bctx->init_digest, bctx->init_digest_len);
	rctx->digest_dma_addr = dma_map_single(crypto_dev->dev, rctx->digest,
					       SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
	rctx->buffer_dma_addr = dma_map_single(crypto_dev->dev, rctx->buffer,
					       rctx->buflen + rctx->block_size, DMA_TO_DEVICE);
	ahash_engine->src_dma = rctx->buffer_dma_addr;
	ahash_engine->src_length = rctx->bufcnt;
	ahash_engine->digeset_dma = rctx->digest_dma_addr;
	return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_transfer);
}

static int aspeed_ahash_g6_update_resume(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");

	dma_unmap_sg(crypto_dev->dev, rctx->src_sg, rctx->src_nents, DMA_TO_DEVICE);
	if (rctx->bufcnt != 0) {
		dma_unmap_single(crypto_dev->dev, rctx->buffer_dma_addr,
				 rctx->buflen + rctx->block_size, DMA_TO_DEVICE);
	}
	dma_unmap_single(crypto_dev->dev, rctx->digest_dma_addr,
			 SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
	// printk("resume digsize\n");
	// for (i = 0; i < rctx->digsize; i++) {
	// 	printk(KERN_CONT "%x ", rctx->digest[i]);
	// }
	// printk("\n");
	// sg_pcopy_to_buffer(rctx->src_sg, rctx->src_nents,
	// 		   rctx->buffer, rctx->total - rctx->offset, rctx->offset);
	scatterwalk_map_and_copy(rctx->buffer, rctx->src_sg,
				 rctx->offset, rctx->total - rctx->offset, 0);
	rctx->bufcnt = rctx->total - rctx->offset;

	rctx->cmd &= ~HASH_CMD_HASH_SRC_SG_CTRL;
	if (rctx->flags & SHA_FLAGS_FINUP) {
		aspeed_crypto_ahash_fill_padding(rctx);
		// printk("after padding\n");
		// for (i = 0; i < rctx->bufcnt; i++) {
		// 	printk(KERN_CONT "%x ", rctx->buffer[i]);
		// }
		// printk("\n");
		rctx->digest_dma_addr = dma_map_single(crypto_dev->dev, rctx->digest,
						       SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
		rctx->buffer_dma_addr = dma_map_single(crypto_dev->dev, rctx->buffer,
						       rctx->buflen + rctx->block_size, DMA_TO_DEVICE);
		ahash_engine->src_dma = rctx->buffer_dma_addr;
		ahash_engine->src_length = rctx->bufcnt;
		ahash_engine->digeset_dma = rctx->digest_dma_addr;
		if (rctx->flags & SHA_FLAGS_HMAC)
			return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_hmac_resume); //TODO
		else
			return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_transfer);
	}
	aspeed_ahash_complete(crypto_dev, 0);
	return 0;
}

static int aspeed_ahash_update_resume(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");
	dma_unmap_single(crypto_dev->dev, rctx->digest_dma_addr,
			 SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
	if (rctx->flags & SHA_FLAGS_FINUP) {
		/* no final() after finup() */
		// printk("rctx->bufcnt: %d\n",rctx->bufcnt);
		aspeed_crypto_ahash_fill_padding(rctx);
		// printk("rctx->buffer\n");
		// for (i = 0; i < rctx->bufcnt; i++) {
		// 	printk("%x ", rctx->buffer[i]);
		// }
		rctx->buffer_dma_addr = dma_map_single(crypto_dev->dev, rctx->buffer,
						       rctx->buflen + rctx->block_size, DMA_TO_DEVICE);
		rctx->digest_dma_addr = dma_map_single(crypto_dev->dev, rctx->digest,
						       SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
		ahash_engine->src_dma = rctx->buffer_dma_addr;
		ahash_engine->src_length = rctx->bufcnt;
		ahash_engine->digeset_dma = rctx->digest_dma_addr;
		if (rctx->flags & SHA_FLAGS_HMAC)
			return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_hmac_resume);
		else
			return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_transfer);
	}


	return aspeed_ahash_complete(crypto_dev, 0);
}

static int aspeed_ahash_req_update(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");
	if (crypto_dev->version == 6) {
		rctx->cmd |= HASH_CMD_HASH_SRC_SG_CTRL;
		aspeed_ahash_append_sg_map(crypto_dev);
		return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_g6_update_resume);
	}
	aspeed_ahash_dma_prepare(crypto_dev);
	return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_update_resume);
}

static int aspeed_ahash_req_final(struct aspeed_crypto_dev *crypto_dev)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");
	aspeed_crypto_ahash_fill_padding(rctx);
	// aspeed_ahash_dma_prepare(crypto_dev);
	rctx->digest_dma_addr = dma_map_single(crypto_dev->dev, rctx->digest,
					       SHA512_DIGEST_SIZE, DMA_BIDIRECTIONAL);
	rctx->buffer_dma_addr = dma_map_single(crypto_dev->dev, rctx->buffer,
					       rctx->buflen + rctx->block_size, DMA_TO_DEVICE);
	ahash_engine->src_dma = rctx->buffer_dma_addr;
	ahash_engine->src_length = rctx->bufcnt;
	ahash_engine->digeset_dma = rctx->digest_dma_addr;

	if (rctx->flags & SHA_FLAGS_HMAC)
		return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_hmac_resume);

	return aspeed_crypto_ahash_trigger(crypto_dev, aspeed_ahash_transfer);
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


	if (rctx->op == SHA_OP_UPDATE)
		aspeed_ahash_req_update(crypto_dev);
	else if (rctx->op == SHA_OP_FINAL)
		aspeed_ahash_req_final(crypto_dev);

	return ret;
}

int aspeed_crypto_ahash_trigger(struct aspeed_crypto_dev *crypto_dev,
				aspeed_crypto_fn_t resume)
{
	struct aspeed_engine_ahash *ahash_engine = &crypto_dev->ahash_engine;
	struct ahash_request *req = ahash_engine->ahash_req;
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);

	AHASH_DBG("\n");
#ifdef CONFIG_CRYPTO_DEV_ASPEED_AHASH_INT
	rctx->cmd |= HASH_CMD_INT_ENABLE;
	ahash_engine->resume = resume;
#endif
	aspeed_crypto_write(crypto_dev, ahash_engine->src_dma, ASPEED_HACE_HASH_SRC);
	aspeed_crypto_write(crypto_dev, ahash_engine->digeset_dma, ASPEED_HACE_HASH_DIGEST_BUFF);
	aspeed_crypto_write(crypto_dev, ahash_engine->digeset_dma, ASPEED_HACE_HASH_KEY_BUFF);
	aspeed_crypto_write(crypto_dev, ahash_engine->src_length, ASPEED_HACE_HASH_DATA_LEN);
	aspeed_crypto_write(crypto_dev, rctx->cmd, ASPEED_HACE_HASH_CMD);
	// rctx->bufcnt = 0;
#ifndef CONFIG_CRYPTO_DEV_ASPEED_AHASH_INT
	while (aspeed_crypto_read(crypto_dev, ASPEED_HACE_STS) & HACE_HASH_BUSY);
	aspeed_crypto_write(crypto_dev, sts, ASPEED_HACE_STS);
	resume(crypto_dev);
	return 0;
#endif
	return -EINPROGRESS;
}

static int aspeed_sham_update(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	struct aspeed_crypto_dev *crypto_dev = tctx->crypto_dev;

	AHASH_DBG("\n");
	// printk("req->nbytes: %d\n", req->nbytes);
	rctx->total = req->nbytes;
	rctx->src_sg = req->src;
	rctx->offset = 0;
	rctx->src_nents = sg_nents(req->src); //g6
	rctx->op = SHA_OP_UPDATE;

	rctx->digcnt[0] += rctx->total;
	if (rctx->digcnt[0] < rctx->total)
		rctx->digcnt[1]++;

	if (rctx->bufcnt + rctx->total < rctx->block_size) {
		// sg_copy_to_buffer(rctx->src_sg, rctx->src_nents,
		// 		  rctx->buffer + rctx->bufcnt, rctx->total);
		scatterwalk_map_and_copy(rctx->buffer + rctx->bufcnt, rctx->src_sg,
					 rctx->offset, rctx->total, 0);
		rctx->bufcnt += rctx->total;
		// aspeed_sham_append_sg(rctx);
		return 0;
	}

	return aspeed_crypto_ahash_handle_queue(crypto_dev, &req->base);
}

static int aspeed_sham_shash_digest(struct crypto_shash *tfm, u32 flags,
				    const u8 *data, unsigned int len, u8 *out)
{
	SHASH_DESC_ON_STACK(shash, tfm);

	shash->tfm = tfm;
	shash->flags = flags & CRYPTO_TFM_REQ_MAY_SLEEP;

	return crypto_shash_digest(shash, data, len, out);
}

static int aspeed_sham_final(struct ahash_request *req)
{
	struct crypto_ahash *tfm = crypto_ahash_reqtfm(req);
	struct aspeed_sham_reqctx *rctx = ahash_request_ctx(req);
	struct aspeed_sham_ctx *tctx = crypto_ahash_ctx(tfm);
	struct aspeed_crypto_dev *crypto_dev = tctx->crypto_dev;

	AHASH_DBG("req->nbytes %d , rctx->total %d\n", req->nbytes, rctx->total);
	rctx->op = SHA_OP_FINAL;

	// aspeed_crypto_ahash_fill_padding(rctx);

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
	struct aspeed_sha_hmac_ctx *bctx = tctx->base;

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
	case SHA384_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_SHA512_SER | HASH_CMD_SHA384 |
			     HASH_CMD_SHA_SWAP;
		rctx->flags |= SHA_FLAGS_SHA384;
		rctx->digsize = SHA384_DIGEST_SIZE;
		rctx->block_size = SHA384_BLOCK_SIZE;
		memcpy(rctx->digest, sha384_iv, 64);
		break;
	case SHA512_DIGEST_SIZE:
		rctx->cmd |= HASH_CMD_SHA512_SER | HASH_CMD_SHA512 |
			     HASH_CMD_SHA_SWAP;
		rctx->flags |= SHA_FLAGS_SHA512;
		rctx->digsize = SHA512_DIGEST_SIZE;
		rctx->block_size = SHA512_BLOCK_SIZE;
		memcpy(rctx->digest, sha512_iv, 64);
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
	rctx->buflen = SHA512_BLOCK_SIZE;
	//hmac cmd
	if (tctx->flags & SHA_FLAGS_HMAC) {
		rctx->digcnt[0] = rctx->block_size;
		rctx->bufcnt = rctx->block_size;
		memcpy(rctx->buffer, bctx->ipad, rctx->block_size);
		rctx->flags |= SHA_FLAGS_HMAC;
	}
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
	struct aspeed_sha_hmac_ctx *bctx = tctx->base;
	int bs = crypto_shash_blocksize(bctx->shash);
	int ds = crypto_shash_digestsize(bctx->shash);
	int err = 0, i;

	AHASH_DBG("\n");
	if (keylen > bs) {
		err = aspeed_sham_shash_digest(bctx->shash,
					       crypto_shash_get_flags(bctx->shash),
					       key, keylen, bctx->ipad);
		if (err)
			return err;
		keylen = ds;
	} else {
		memcpy(bctx->ipad, key, keylen);
	}
	memset(bctx->ipad + keylen, 0, bs - keylen);
	memcpy(bctx->opad, bctx->ipad, bs);

	for (i = 0; i < bs; i++) {
		bctx->ipad[i] ^= 0x36;
		bctx->opad[i] ^= 0x5c;
	}

	return err;
}

static int aspeed_sham_cra_init_alg(struct crypto_tfm *tfm, const char *alg_base)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);
	struct aspeed_crypto_alg *algt;
	struct ahash_alg *alg = __crypto_ahash_alg(tfm->__crt_alg);

	algt = container_of(alg, struct aspeed_crypto_alg, alg.ahash);
	tctx->crypto_dev = algt->crypto_dev;
	tctx->flags = 0;

	AHASH_DBG("%s crypto dev %x \n", crypto_tfm_alg_name(tfm),
		  (u32)tctx->crypto_dev);

	crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct aspeed_sham_reqctx));

	if (alg_base) {
		struct aspeed_sha_hmac_ctx *bctx = tctx->base;
		tctx->flags |= SHA_FLAGS_HMAC;
		bctx->shash = crypto_alloc_shash(alg_base, 0,
						 CRYPTO_ALG_NEED_FALLBACK);
		if (IS_ERR(bctx->shash)) {
			pr_err("aspeed-sham: base driver '%s' "
			       "could not be loaded.\n", alg_base);
			return PTR_ERR(bctx->shash);
		}
	}

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

static int aspeed_sham_cra_sha384_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha384");
}

static int aspeed_sham_cra_sha512_init(struct crypto_tfm *tfm)
{
	return aspeed_sham_cra_init_alg(tfm, "sha512");
}

static void aspeed_sham_cra_exit(struct crypto_tfm *tfm)
{
	struct aspeed_sham_ctx *tctx = crypto_tfm_ctx(tfm);

	AHASH_DBG("\n");

	if (tctx->flags & SHA_FLAGS_HMAC) {
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
	{
		.alg.ahash = {
			.init	= aspeed_sham_init,
			.update	= aspeed_sham_update,
			.final	= aspeed_sham_final,
			.finup	= aspeed_sham_finup,
			.digest	= aspeed_sham_digest,
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = MD5_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "hmac(md5)",
					.cra_driver_name	= "aspeed-hmac-md5",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= MD5_HMAC_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx) + sizeof(struct aspeed_sha_hmac_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_md5_init,
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
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA1_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "hmac(sha1)",
					.cra_driver_name	= "aspeed-hmac-sha1",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA1_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx) + sizeof(struct aspeed_sha_hmac_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_sha1_init,
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
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA224_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "hmac(sha224)",
					.cra_driver_name	= "aspeed-hmac-sha224",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA224_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx) + sizeof(struct aspeed_sha_hmac_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_sha224_init,
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
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA256_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "hmac(sha256)",
					.cra_driver_name	= "aspeed-hmac-sha256",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA256_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx) + sizeof(struct aspeed_sha_hmac_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_sha256_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
};
struct aspeed_crypto_alg aspeed_ahash_algs_g6[] = {
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
				.digestsize = SHA384_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "sha384",
					.cra_driver_name	= "aspeed-sha384",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA384_BLOCK_SIZE,
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
				.digestsize = SHA512_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "sha512",
					.cra_driver_name	= "aspeed-sha512",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA512_BLOCK_SIZE,
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
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA384_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "hmac(sha384)",
					.cra_driver_name	= "aspeed-hmac-sha384",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA384_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx) + sizeof(struct aspeed_sha_hmac_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_sha384_init,
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
			.setkey	= aspeed_sham_setkey,
			.export	= aspeed_sham_export,
			.import	= aspeed_sham_import,
			.halg = {
				.digestsize = SHA512_DIGEST_SIZE,
				.statesize = sizeof(struct aspeed_sham_reqctx),
				.base = {
					.cra_name		= "hmac(sha512)",
					.cra_driver_name	= "aspeed-hmac-sha512",
					.cra_priority		= 300,
					.cra_flags		= CRYPTO_ALG_TYPE_AHASH |
					CRYPTO_ALG_ASYNC |
					CRYPTO_ALG_KERN_DRIVER_ONLY,
					.cra_blocksize		= SHA512_BLOCK_SIZE,
					.cra_ctxsize		= sizeof(struct aspeed_sham_ctx) + sizeof(struct aspeed_sha_hmac_ctx),
					.cra_alignmask		= 0,
					.cra_module		= THIS_MODULE,
					.cra_init		= aspeed_sham_cra_sha512_init,
					.cra_exit		= aspeed_sham_cra_exit,
				}
			}
		},
	},
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
	if (crypto_dev->version == 6) {
		for (i = 0; i < ARRAY_SIZE(aspeed_ahash_algs_g6); i++) {
			aspeed_ahash_algs_g6[i].crypto_dev = crypto_dev;
			err = crypto_register_ahash(&aspeed_ahash_algs_g6[i].alg.ahash);
			if (err)
				return err;
		}
	}
	return 0;
}
