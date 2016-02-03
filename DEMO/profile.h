#pragma once

#import <chrono>
#import <string>
#import <iostream>

class profile{
public:
    profile(const char * text = nullptr);
    ~profile();

private:
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    std::string text_;
};

profile::~profile() {
    std::chrono::duration<double> duration_ =
            std::chrono::system_clock::now() - start_time_;
    std::cout << text_ << ": " << duration_.count() << " ms" << std::endl;
}

profile::profile(const char * text) {
    if(text)
        text_ = std::string(text);

    start_time_ = std::chrono::system_clock::now();
}
