/**
 * @file md5.h
 * @The header file of md5.
 * @author Jiewei Wei
 * @mail weijieweijerry@163.com
 * @github https://github.com/JieweiWei
 * @data Oct 19 2014
 *
 */


/* Define of btye.*/
typedef unsigned char Byte;
/* Define of Byte. */
typedef unsigned int bit32;

class MD5 {
public:
	/* Construct a MD5 object with a string. */
	MD5(const std::string& message);

	/* Generate md5 digest. */
	const Byte* getDigest();

	/* Convert digest to string value */
	std::string toStr();

private:
	/* Initialization the md5 object, processing another message block,
	 * and updating the context.*/
	void init(const Byte* input, size_t len);

	/* MD5 basic transformation. Transforms state based on block. */
	void transform(const Byte block[64]);

	/* Encodes input (usigned long) into output (Byte). */
	void encode(const bit32* input, Byte* output, size_t length);

	/* Decodes input (Byte) into output (usigned long). */
	void decode(const Byte* input, bit32* output, size_t length);

private:
	/* Flag for mark whether calculate finished. */
	bool finished;

	/* state (ABCD). */
	bit32 state[4];

	/* number of bits, low-order word first. */
	bit32 count[2];

	/* input buffer. */
	Byte buffer[64];

	/* message digest. */
	Byte digest[16];

	/* padding for calculate. */
	static const Byte PADDING[64];

	/* Hex numbers. */
	static const char HEX_NUMBERS[16];
};


namespace para
{
	std::string getMD5(const std::string& source);
}

