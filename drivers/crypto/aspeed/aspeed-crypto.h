#ifndef __ASPEED_CRYPTO_H__
#define __ASPEED_CRYPTO_H__

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/fips.h>
#include <linux/dma-mapping.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/kpp.h>
#include <crypto/internal/rsa.h>
#include <crypto/internal/akcipher.h>
#include <crypto/internal/skcipher.h>
#include <crypto/kpp.h>
#include <crypto/dh.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/algapi.h>
#include <crypto/akcipher.h>
#include <crypto/skcipher.h>
#include <crypto/md5.h>
#include <crypto/sha.h>
#include <crypto/ecdh.h>



/* Crypto control registers*/
#define ASPEED_HACE_SRC			0x00
#define ASPEED_HACE_DEST		0x04
#define ASPEED_HACE_CONTEXT		0x08	/* 8 byte aligned*/
#define ASPEED_HACE_DATA_LEN		0x0C
#define ASPEED_HACE_CMD			0x10
#define  HACE_CMD_MBUS_REQ_SYNC_EN	BIT(20) //G6
#define  HACE_CMD_DES_SG_CTRL		BIT(19) //G6
#define  HACE_CMD_SRC_SG_CTRL		BIT(18) //G6
#define  HACE_CMD_SINGLE_DES		0
#define  HACE_CMD_TRIPLE_DES		BIT(17)
#define  HACE_CMD_AES_SELECT		0
#define  HACE_CMD_DES_SELECT		BIT(16)
#define  HACE_CMD_CTR_IV_AES_128	0 //G6
#define  HACE_CMD_CTR_IV_DES_64		0 //G6
#define  HACE_CMD_CTR_IV_AES_96		(0x1 << 14) //G6
#define  HACE_CMD_CTR_IV_DES_32		(0x1 << 14) //G6
#define  HACE_CMD_CTR_IV_AES_64		(0x2 << 14) //G6
#define  HACE_CMD_CTR_IV_AES_32		(0x3 << 14) //G6
#define  HACE_CMD_AES_KEY_HW_EXP	BIT(13) //G6
#define  HACE_CMD_ISR_EN		BIT(12)
#define  HACE_CMD_RI_WO_DATA_ENABLE	(0)
#define  HACE_CMD_RI_WO_DATA_DISABLE	BIT(11)
#define  HACE_CMD_CONTEXT_LOAD_ENABLE	(0)
#define  HACE_CMD_CONTEXT_LOAD_DISABLE	BIT(10)
#define  HACE_CMD_CONTEXT_SAVE_ENABLE	(0)
#define  HACE_CMD_CONTEXT_SAVE_DISABLE	BIT(9)
#define  HACE_CMD_AES			(0)
#define  HACE_CMD_DES			(0)
#define  HACE_CMD_RC4			BIT(8)
#define  HACE_CMD_DECRYPT		(0)
#define  HACE_CMD_ENCRYPT		BIT(7)
#define  HACE_CMD_ECB			(0)
#define  HACE_CMD_CBC			(0x1 << 4)
#define  HACE_CMD_CFB			(0x2 << 4)
#define  HACE_CMD_OFB			(0x3 << 4)
#define  HACE_CMD_CTR			(0x4 << 4)
#define  HACE_CMD_GCM			(0x5 << 4) //G6
#define  HACE_CMD_AES128		(0)
#define  HACE_CMD_AES192		(0x1 << 2)
#define  HACE_CMD_AES256		(0x2 << 2)
#define  HACE_CMD_OP_CASCADE		(0x3)
#define  HACE_CMD_OP_INDEPENDENT	(0x1)
#define ASPEED_HACE_GCM_TAG		0x14 //G6
#define ASPEED_HACE_TAG			0x18
#define ASPEED_HACE_GCM_ADD_LEN		0x18 //G6
#define ASPEED_HACE_STS			0x1C
#define  HACE_RSA_ISR			BIT(13)
#define  HACE_CRYPTO_ISR		BIT(12)
#define  HACE_HASH_ISR			BIT(9)
#define  HACE_RSA_BUSY			BIT(2)
#define  HACE_CRYPTO_BUSY		BIT(1)
#define  HACE_HASH_BUSY			BIT(0)
#define ASPEED_HACE_HASH_SRC		0x20
#define ASPEED_HACE_HASH_DIGEST_BUFF	0x24
#define ASPEED_HACE_HASH_KEY_BUFF	0x28	/* 64 byte aligned*/
#define ASPEED_HACE_HASH_DATA_LEN	0x2C
#define ASPEED_HACE_HASH_CMD		0x30
#define  HASH_CMD_INT_ENABLE		BIT(9)
#define  HASH_CMD_INT_DISABLE		(0)
#define  HASH_CMD_ACC_MODE		BIT(8)
#define  HASH_CMD_HMAC			BIT(7)
#define  HASH_CMD_MD5			(0)
#define  HASH_CMD_SHA1			(0x2 << 4)
#define  HASH_CMD_SHA224		(0x4 << 4)
#define  HASH_CMD_SHA256		(0x5 << 4)
#define  HASH_CMD_SHA512_SER		(0x6 << 4)
#define  HASH_CMD_MD5_SWAP		(0x1 << 2)
#define  HASH_CMD_SHA_SWAP		(0x1 << 3)
#define  HASH_CMD_CASCADED_CRYPTO_FIRST	(2)
#define  HASH_CMD_CASCADED_HASH_FIRST	(3)
#define ASPEED_HACE_RSA_MD_EXP_BIT	0x40
#define ASPEED_HACE_RSA_CMD		0x4C
#define  RSA_CMD_INT_ENABLE		BIT(13)
#define  RSA_CMD_SRAM_ENGINE_ACCESSABLE BIT(12)
#define  RSA_CMD_FIRE			BIT(11)
#define ASPEED_HACE_CMD_QUEUE		0x50
#define ASPEED_HACE_CMD_QUEUE_EP	0x54
#define ASPEED_HACE_CMD_QUEUE_WP	0x58
#define ASPEED_HACE_CMD_QUEUE_RP	0x5C
#define ASPEED_HACE_ENG_FEATURE		0x60

#define ASPEED_CRYPTO_G6		BIT(0)
#define ASPEED_CRYPTO_G6_RSA_BUFF_SIZE	508
#define ASPEED_CRYPTO_RSA_BUFF_SIZE	508

#define HACE_CMD_IV_REQUIRE		(HACE_CMD_CBC | HACE_CMD_CFB | \
					 HACE_CMD_OFB | HACE_CMD_CTR)

#define ASPEED_EUCLID_CTX_LEN		13312
#define ASPEED_EUCLID_LEN		1024
#define ASPEED_EUCLID_A			0
#define ASPEED_EUCLID_B			ASPEED_EUCLID_LEN * 1
#define ASPEED_EUCLID_Q			ASPEED_EUCLID_LEN * 2
#define ASPEED_EUCLID_R			ASPEED_EUCLID_LEN * 3
#define ASPEED_EUCLID_X			ASPEED_EUCLID_LEN * 4
#define ASPEED_EUCLID_Y			ASPEED_EUCLID_LEN * 5
#define ASPEED_EUCLID_LX		ASPEED_EUCLID_LEN * 6
#define ASPEED_EUCLID_LY		ASPEED_EUCLID_LEN * 7
#define ASPEED_EUCLID_T			ASPEED_EUCLID_LEN * 8
#define ASPEED_EUCLID_D1		ASPEED_EUCLID_LEN * 9
#define ASPEED_EUCLID_S			ASPEED_EUCLID_LEN * 10
#define ASPEED_EUCLID_N			ASPEED_EUCLID_LEN * 11
#define ASPEED_EUCLID_NP		ASPEED_EUCLID_LEN * 12

#define CRYPTO_FLAGS_BUSY 		BIT(1)

#define SHA_BUFFER_LEN			128//(PAGE_SIZE/16)

#define SHA_OP_UPDATE			1
#define SHA_OP_FINAL			2

#define SHA_FLAGS_HMAC			BIT(1)

#define SHA_FLAGS_MD5			BIT(17)
#define SHA_FLAGS_SHA1			BIT(18)
#define SHA_FLAGS_SHA224		BIT(19)
#define SHA_FLAGS_SHA256		BIT(20)
#define SHA_FLAGS_SHA384		BIT(21)
#define SHA_FLAGS_SHA512		BIT(22)

#define SHA_FLAGS_FINUP			BIT(23)

struct aspeed_crypto_dev;

typedef int (*aspeed_crypto_fn_t)(struct aspeed_crypto_dev *);


/******************************************************************************/
/* skcipher */
//g6
struct aspeed_sg_list {
	u32 len;
	u32 phy_addr;
};

struct aspeed_engine_skcipher {
	struct crypto_queue		queue;
	struct tasklet_struct		done_task;
	// struct tasklet_struct		queue_task;
	bool				is_async;
	spinlock_t			lock;
	aspeed_crypto_fn_t		resume;
	unsigned long			flags;

	struct skcipher_request		*sk_req;
	void				*cipher_addr;
	dma_addr_t			cipher_dma_addr;

	void				*dst_sg_addr; //g6
	dma_addr_t			dst_sg_dma_addr; //g6
};
//tctx
struct aspeed_cipher_ctx {
	struct aspeed_crypto_dev	*crypto_dev;
	u8				*iv;
	bool				con;
	int 				key_len;
	int 				enc_cmd;
	void				*cipher_key;
	dma_addr_t			cipher_key_dma;
};
/******************************************************************************/
/* sha and md5 */

struct aspeed_engine_ahash {
	struct crypto_queue		queue;
	struct tasklet_struct		done_task;
	// struct tasklet_struct		queue_task;
	bool				is_async;
	spinlock_t			lock;
	aspeed_crypto_fn_t		resume;
	unsigned long			flags;

	struct ahash_request		*ahash_req;
};
//hmac tctx
struct aspeed_sha_hmac_ctx {
	struct crypto_shash *shash;
	u8 ipad[SHA512_BLOCK_SIZE];
	u8 opad[SHA512_BLOCK_SIZE];
	const u32 *init_digest;
};
//sha and md5 tctx
struct aspeed_sham_ctx {
	struct aspeed_crypto_dev	*crypto_dev;
	unsigned long			flags; //hmac flag

	/* fallback stuff */
	struct aspeed_sha_hmac_ctx	base[0];		//for hmac
};
//rctx, state
struct aspeed_sham_reqctx {
	unsigned long		flags;	//final update flag should no use
	unsigned long		op;	//final or update
	u32			cmd;	//trigger cmd

	u8	digest[SHA512_DIGEST_SIZE] __aligned(64);  //digest result
	u64	digcnt[2];  //total length
	size_t	digsize; //digest size
	dma_addr_t	src_dma_addr;  //input src dma address
	dma_addr_t	digest_dma_addr;  //output digesft result dma address

	/* walk state */
	struct scatterlist	*src_sg;
	unsigned int		offset;	/* offset in current sg */
	unsigned int		total;	/* per update length*/

	size_t 		block_size;

	dma_addr_t	buffer_dma_addr;
	size_t		bufcnt;  //buffer counter
	size_t		buflen;  //buffer length
	u8		buffer[SHA_BUFFER_LEN + SHA512_BLOCK_SIZE];
};

/******************************************************************************/
/* akcipher rsa */
struct aspeed_engine_akcipher {
	void __iomem			*rsa_buff;
	unsigned long			rsa_max_buf_len;
	struct akcipher_request		*akcipher_req;

};

/**
 * aspeed_rsa_key - ASPEED RSA key structure. Keys are allocated in DMA zone.
 * @n           : RSA modulus raw byte stream
 * @e           : RSA public exponent raw byte stream
 * @d           : RSA private exponent raw byte stream
 * @np          : raw byte stream for Montgomery's method, length equal to n_sz
 * @n_sz        : length in bytes of RSA modulus n
 * @e_sz        : length in bytes of RSA public exponent
 * @d_sz        : length in bytes of RSA private exponent
 */
struct aspeed_rsa_key {
	u8 *n;
	u8 *e;
	u8 *d;
	size_t n_sz;
	size_t e_sz;
	size_t d_sz;
	int nm;
	int ne;
	int nd;
	int dwm;
	int mdwm;
	u8 *np;
};

struct aspeed_rsa_ctx {
	struct aspeed_crypto_dev *crypto_dev;
	struct aspeed_rsa_key key;
	u8 *euclid_ctx;
	int enc;
};

/*************************************************************************************/

struct aspeed_ecdh_ctx {
	struct aspeed_crypto_dev	*crypto_dev;
	const u8 			*public_key;
	unsigned int 			curve_id;
	size_t				n_sz;
	u8				private_key[256];
};

/*************************************************************************************/

struct aspeed_crypto_dev {
	void __iomem			*regs;
	struct device			*dev;
	int 				irq;
	struct clk			*yclk;
	struct clk			*rsaclk;
	unsigned long			version;
	struct aspeed_engine_skcipher	sk_engine;
	struct aspeed_engine_ahash	ahash_engine;
};


struct aspeed_crypto_alg {
	struct aspeed_crypto_dev	*crypto_dev;
	union {
		struct skcipher_alg	skcipher;
		struct ahash_alg	ahash;
		struct kpp_alg 		kpp;
		struct akcipher_alg 	akcipher;
	} alg;
};

static inline void
aspeed_crypto_write(struct aspeed_crypto_dev *crypto, u32 val, u32 reg)
{
	// printk("write : val: %x , reg : %x \n",val,reg);
	writel(val, crypto->regs + reg);
}

static inline u32
aspeed_crypto_read(struct aspeed_crypto_dev *crypto, u32 reg)
{
#if 0
	u32 val = readl(crypto->regs + reg);
	printk("R : reg %x , val: %x \n", reg, val);
	return val;
#else
	return readl(crypto->regs + reg);
#endif
}

extern int aspeed_crypto_skcipher_trigger(struct aspeed_crypto_dev *aspeed_crypto);

extern int aspeed_crypto_ahash_trigger(struct aspeed_crypto_dev *aspeed_crypto,
				       aspeed_crypto_fn_t resume);
extern int aspeed_crypto_ahash_handle_queue(struct aspeed_crypto_dev *aspeed_crypto, struct crypto_async_request *areq);

extern int aspeed_crypto_rsa_trigger(struct aspeed_crypto_dev *aspeed_crypto);

extern int aspeed_register_skcipher_algs(struct aspeed_crypto_dev *crypto_dev);
extern int aspeed_register_ahash_algs(struct aspeed_crypto_dev *crypto_dev);
extern int aspeed_register_akcipher_algs(struct aspeed_crypto_dev *crypto_dev);
extern int aspeed_register_kpp_algs(struct aspeed_crypto_dev *crypto_dev);

#endif
