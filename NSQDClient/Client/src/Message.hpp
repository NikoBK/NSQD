#ifndef MESSAGE_H
#define MESSAGE_H

#include <vector>
#include <WinSock2.h>
#include <string>
#include <iterator>
#include <stdexcept>

class Encoder
{
public:
    Encoder()
    {
        // Reserve enough space for the size of this buffer
        WriteInt(0);
    }

    void WriteBoolean(bool value)
    {
        Write((char*)&value, sizeof(bool));
    }

    void WriteByte(char value)
    {
        Write((char*)&value, sizeof(char));
    }

    void WriteShort(short value)
    {
        value = htons(value);
        Write((char*)&value, sizeof(short));
    }

    void WriteInt(int value)
    {
        value = htonl(value);
        Write((char*)&value, sizeof(int));
    }

    void WriteFloat(float value)
    {
        unsigned int val = htonf(value);
        Write((char*)&val, sizeof(float));
    }

    void WriteString(const std::string& value)
    {
        short size = value.length();
        WriteShort(size);
        Write((char*)value.c_str(), size);
    }

    const char* buffer() const
    {
        // get the position and then copy to the front of the _buffer
        int length = htonl(_position);
        memcpy((char*)_buffer.data(), &length, sizeof(int));
        return _buffer.data();
    }

    const int size() const {
        return _buffer.size();
    }

private:
    void Write(char* data, unsigned int size)
    {
        // Reserve space in the buffer to avoid reallocations
        _buffer.reserve(_buffer.size() + size);

        // Copy data into buffer using std::copy
        std::copy(data, data + size, std::back_inserter(_buffer));

        // Update position
        _position += size;
    }

private:
    int _position;
    std::vector<char> _buffer;
};

class Decoder {
public:
    Decoder(const char* data, int size)
        : _buffer(data, data + size), _position(0)
    {
    }

    void ReadBoolean(bool* value) {
        Read(reinterpret_cast<char*>(value), sizeof(bool));
    }

    void ReadByte(unsigned char* value) {
        Read(reinterpret_cast<char*>(value), sizeof(unsigned char));
    }

    void ReadShort(short* value) {
        Read(reinterpret_cast<char*>(value), sizeof(short));
        *value = ntohs(*value);
    }

    void ReadInt(int* value) {
        Read(reinterpret_cast<char*>(value), sizeof(int));
        *value = ntohl(*value);
    }

    void ReadFloat(float* value) {
        unsigned int temp;
        Read(reinterpret_cast<char*>(&temp), sizeof(float));
        *value = ntohf(temp);
    }

    void ReadString(std::string* value) {
        short size;
        ReadShort(&size);
        value->assign(&_buffer[_position], size);
        _position += size;
    }

private:
    void Read(char* data, unsigned int size) {
        if (_position + size > _buffer.size()) {
            throw std::runtime_error("Not enough data in buffer");
        }
        std::memcpy(data, &_buffer[_position], size);
        _position += size;
    }

private:
    std::vector<char> _buffer;
    int _position;
};

// define a message id here

#define TEST_MESSAGE_ID 0
#define RPY_MESSAGE_ID 1

struct Message
{
    virtual void encode(Encoder& encoder) = 0;
    virtual void decode(Decoder& decoder) = 0;
};

struct TestMessage : public Message
{
    bool a;
    unsigned char b;
    short c;
    int d;
    float e;
    std::string f;

    virtual void encode(Encoder& encoder) override
    {
        encoder.WriteByte(TEST_MESSAGE_ID);
        encoder.WriteBoolean(a);
        encoder.WriteByte(b);
        encoder.WriteShort(c);
        encoder.WriteInt(d);
        encoder.WriteFloat(e);
        encoder.WriteString(f);
    }

    virtual void decode(Decoder& decoder) override
    {
        decoder.ReadBoolean(&a);
        decoder.ReadByte(&b);
        decoder.ReadShort(&c);
        decoder.ReadInt(&d);
        decoder.ReadFloat(&e);
        decoder.ReadString(&f);
    }
};

struct RPYMessage : public Message
{
    float roll;
    float pitch;
    float yaw;

    virtual void encode(Encoder& encoder) override
    {
        encoder.WriteByte(RPY_MESSAGE_ID);
        encoder.WriteFloat(roll);
        encoder.WriteFloat(pitch);
        encoder.WriteFloat(yaw);
    }

    virtual void decode(Decoder& decoder) override
    {
        decoder.ReadFloat(&roll);
        decoder.ReadFloat(&pitch);
        decoder.ReadFloat(&yaw);
    }
};

#endif // !MESSAGE_H
