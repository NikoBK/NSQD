#ifndef MESSAGE_H
#define MESSAGE_H

#include <vector>
#include <string.h>
#include <iterator>
#include <stdexcept>
#include "../include/Constants.hpp"

class Encoder {
public:
    Encoder() : _position(0) {
        // Reserve enough space for the size of this buffer
        WriteInt(0);
    }

    void WriteBoolean(bool value) {
        Write((char*)&value, sizeof(bool));
    }

    void WriteByte(char value) {
        Write((char*)&value, sizeof(char));
    }

    void WriteShort(short value) {
        value = htons(value);
        Write((char*)&value, sizeof(short));
    }

    void WriteInt(int value) {
        value = htonl(value);
        Write((char*)&value, sizeof(int));
    }

    void WriteFloat(float value) {
        unsigned int val = htonl(value);
        Write((char*)&val, sizeof(float));
    }

    void WriteString(const std::string& value) {
        short size = value.length();
        WriteShort(size);
        Write((char*)value.c_str(), size);
    }

    const char* buffer() const {
        // get the position and then copy to the front of the _buffer
        int length = htonl(_position);
        //std::cout << "length is: " << length << std::endl;
        memcpy((char*)_buffer.data(), &length, sizeof(int));
        return _buffer.data();
    }

    const int size() const {
        return _buffer.size();
    }

private:
    void Write(char *data, unsigned int size) {
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
    Decoder(const char* data, int size) : _buffer(data, data + size), _position(0)
    { }

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
        *value = ntohl(temp);
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
        memcpy(data, &_buffer[_position], size);
        _position += size;
    }

private:
    std::vector<char> _buffer;
    int _position;
};

struct Message {
    virtual void encode(Encoder& encoder) = 0;
    virtual void decode(Decoder& decoder) = 0;
};

struct ErrorMessage : public Message
{
    std::string text;

    virtual void encode(Encoder& encoder) override {
        encoder.WriteByte(ERROR_MSG);
        encoder.WriteString(text);
    }

    virtual void decode(Decoder& decoder) override {
        decoder.ReadString(&text);
    }
};

struct UpdateMessage : public Message
{
    float roll;
    float pitch;
    float yaw;
    float thrust;
    float lat;
    float lon;
    float alt;
    int state;

    virtual void encode(Encoder& encoder) override
    {
        encoder.WriteByte(UPDATE_MSG);
        encoder.WriteFloat(roll);
        encoder.WriteFloat(pitch);
        encoder.WriteFloat(yaw);
		encoder.WriteFloat(thrust);
        encoder.WriteFloat(lat);
        encoder.WriteFloat(lon);   
		encoder.WriteFloat(alt);  
        encoder.WriteInt(state);  
    }

    virtual void decode(Decoder& decoder) override
    {
        decoder.ReadFloat(&roll);
        decoder.ReadFloat(&pitch);
        decoder.ReadFloat(&yaw);
		decoder.ReadFloat(&thrust);
        decoder.ReadFloat(&lat);
        decoder.ReadFloat(&lon);   
		decoder.ReadFloat(&alt); 
        decoder.ReadInt(&state);
    }
};

struct SetAuthorityMessage : public Message 
{
    virtual void encode(Encoder& encoder) override {
        encoder.WriteByte(SET_AUTH_MSG);
    }

    virtual void decode(Decoder& decoder) override { }
};

struct ArmMessage : public Message 
{
    /* true: armed, false: disarmed*/
    bool status;

    virtual void encode(Encoder& encoder) override {
        encoder.WriteByte(ARM_MSG);
        encoder.WriteBoolean(status);
    }

    virtual void decode(Decoder& decoder) override { 
        decoder.ReadBoolean(&status);
    }
};

struct TakeoffMessage : public Message 
{
    virtual void encode(Encoder& encoder) override {
        encoder.WriteByte(TAKEOFF_MSG);
    }

    virtual void decode(Decoder& decoder) override { }
};

struct LandMessage : public Message 
{
    virtual void encode(Encoder& encoder) override {
        encoder.WriteByte(LAND_MSG);
    }

    virtual void decode(Decoder& decoder) override { }
};

struct SetPIDMessage : public Message 
{
    std::string kp; // Proportional
    std::string ki; // Integral
    std::string kd; // Derivative

    // Flag for Roll (0), Pitch (1), Thrust (2) and Yaw (3) PID regulator.
    int flag;

    virtual void encode(Encoder& encoder) override {
        encoder.WriteByte(SET_PID_MSG);
        encoder.WriteString(kp);
        encoder.WriteString(ki);
        encoder.WriteString(kd);
        encoder.WriteInt(flag);
    }

    virtual void decode(Decoder& decoder) override { 
        decoder.ReadString(&kp);
        decoder.ReadString(&ki);
        decoder.ReadString(&kd);
        decoder.ReadInt(&flag);
    }
};

struct SetRPYTFFMessage : public Message
{
    // TODO: Convert back to floats when float read/write works.
    /*float*/std::string roll;
    /*float*/std::string pitch;
    /*float*/std::string yaw;
    /*float*/std::string thrust;
    int flag;
    std::string fileName;

    virtual void encode(Encoder& encoder) override {
        encoder.WriteByte(SET_RPYTFF_MSG);
        encoder./*WriteFloat*/WriteString(roll);
        encoder./*WriteFloat*/WriteString(pitch);
        encoder./*WriteFloat*/WriteString(yaw);
        encoder./*WriteFloat*/WriteString(thrust);
        encoder.WriteInt(flag);
        encoder.WriteString(fileName);
    }

    virtual void decode(Decoder& decoder) override {
        decoder./*ReadFloat*/ReadString(&roll);
        decoder./*ReadFloat*/ReadString(&pitch);
        decoder./*ReadFloat*/ReadString(&yaw);
        decoder./*ReadFloat*/ReadString(&thrust);
        decoder.ReadInt(&flag);
        decoder.ReadString(&fileName);
    }
};

struct StopTestMessage : public Message
{
	virtual void encode(Encoder& encoder) override {
		encoder.WriteByte(STOP_TEST_MSG);
	}
	virtual void decode(Decoder& decoder) override { }
};

struct StartImageCaptureMessage : public Message
{
	virtual void encode(Encoder& encoder) override {
		encoder.WriteByte(CAPTURE_IMAGES_MSG);
	}
	virtual void decode(Decoder& decoder) override { }
};

struct StopImageCaptureMessage : public Message
{
	virtual void encode(Encoder& encoder) override {
		encoder.WriteByte(STOP_CAPTURE_IMAGES_MSG);
	}
	virtual void decode(Decoder& decoder) override { }
};

struct SetHoverHeightMessage : public Message
{
	float height;
	std::string fileName;

	virtual void encode(Encoder& encoder) override {
		encoder.WriteByte(SET_HOVERHEIGHT_MSG);
		encoder.WriteFloat(height);
		encoder.WriteString(fileName);
	}
	virtual void decode(Decoder& decoder) override { 
		decoder.ReadFloat(&height);
		decoder.ReadString(&fileName);
	}
};

struct FollowLineMessage : public Message
{
	virtual void encode(Encoder& encoder) override {
		encoder.WriteByte(FOLLOW_LINE_MSG);
	}
	virtual void decode(Decoder& decoder) override { }
};

struct UploadFlightPathMessage : public Message
{
	std::string data;
	
	virtual void encode(Encoder& encoder) override {
		encoder.WriteByte(UPLOAD_FLIGHTPATH_MSG);
		encoder.WriteString(data);
	}
	
	virtual void decode(Decoder& decoder) override {
		decoder.ReadString(&data);
	}
};

#endif // !MESSAGE_H
