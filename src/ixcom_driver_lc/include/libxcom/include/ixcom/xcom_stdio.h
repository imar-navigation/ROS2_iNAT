/*.*******************************************************************
 FILENAME: xcom_stdio.h
 **********************************************************************
 *  PROJECT: iXCOM_SDK
 *
 *
 *---------------------------------------------------------------------
 * 	Copyright 2021, iMAR Navigation
 *---------------------------------------------------------------------
 * 	MODULE DESCRIPTION:
 *
 ---------------------------------------------------------------------*/
#ifndef LIB_IXCOM_XCOM_STDIO_H
#define LIB_IXCOM_XCOM_STDIO_H
#include <cstdio>
#include <ixcom/ixcom.h>
#include <string>
#include <utility>
#include <vector>
namespace xcom {
class StdinReader : public xcom::IReader {
public:
    StdinReader() noexcept = default;
    bool initialize() noexcept override {
        auto ret = std::freopen(nullptr, "rb", stdin);
        if(ret == nullptr) {
            return false;
        }
        if(std::ferror(stdin)) {
            return false;
        }
        return true;
    }
    int32_t read(uint8_t* buffer, std::size_t buffer_length) noexcept override {
        return static_cast<int32_t>(std::fread(reinterpret_cast<void*>(buffer), sizeof(buffer[0]), buffer_length, stdin));
    }
};
class StdoutWriter : public xcom::IWriter {
public:
    StdoutWriter() noexcept = default;
    bool initialize() noexcept override {
        auto ret = std::freopen(nullptr, "wb", stdout);
        if(ret == nullptr) {
            return false;
        }
        if(std::ferror(stdout)) {
            return false;
        }
        return true;
    }
    int32_t write(const uint8_t* buffer, std::size_t buffer_length) noexcept override {
        return static_cast<int32_t>(std::fwrite(buffer, sizeof(buffer[0]), buffer_length, stdout));
    }
};
class FileReader : public xcom::IReader {
public:
    explicit FileReader(std::string&& input_file) noexcept
        : InputFile(input_file) {}
    explicit FileReader(std::string& input_file) noexcept
        : InputFile(std::move(input_file)) {}
    explicit FileReader(const std::string& input_file) noexcept
        : InputFile(input_file) {}
    ~FileReader() override {
        if(_stream != nullptr) {
            std::fclose(_stream);
        }
    }
    bool initialize() noexcept override {
        _stream = std::fopen(InputFile.data(), "rb");
        if(_stream == nullptr) {
            return false;
        }
        return true;
    }
    int32_t read(uint8_t* buffer, std::size_t buffer_length) noexcept override {
        return static_cast<int32_t>(std::fread(buffer, sizeof(buffer[0]), buffer_length, _stream));
    }
private:
    const std::string InputFile;
    std::FILE* _stream = nullptr;
};
class FileWriter : public xcom::IWriter {
public:
    explicit FileWriter(std::string&& input_file) noexcept
        : OutputFile(input_file) {}
    explicit FileWriter(std::string& input_file) noexcept
        : OutputFile(std::move(input_file)) {}
    explicit FileWriter(const std::string& input_file) noexcept
        : OutputFile(input_file) {}
    ~FileWriter() override {
        if(_stream != nullptr) {
            std::fclose(_stream);
        }
    }
    bool initialize() noexcept override {
        _stream = std::fopen(OutputFile.c_str(), "wb");
        if(_stream == nullptr) {
            return false;
        }
        return true;
    }
    void flush() { fflush(_stream); }
    int32_t write(const uint8_t* buffer, std::size_t buffer_length) noexcept override {
        return static_cast<int32_t>(std::fwrite(buffer, sizeof(buffer[0]), buffer_length, _stream));
    }
private:
    const std::string OutputFile;
    std::FILE* _stream = nullptr;
};
class VectorReader : public xcom::IReader {
public:
    explicit VectorReader(const std::vector<uint8_t>& input) noexcept
        : _input(input) {}
    ~VectorReader() override = default;
    bool initialize() noexcept override { return true; }
    int32_t read(uint8_t* buffer, std::size_t buffer_length) noexcept override {
        if(_input.empty()) {
            return -1;
        }
        auto len = buffer_length;
        if(buffer_length > _input.size()) {
            len = _input.size();
        }
        for(std::size_t idx = 0; idx < len; idx++) {
            buffer[idx] = _input[0];
            _input.erase(_input.begin());
        }
        return static_cast<int32_t>(len);
    }
private:
    std::vector<uint8_t> _input{};
};
}  // namespace xcom
#endif  // LIB_IXCOM_XCOM_STDIO_H
