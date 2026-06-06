#pragma once

#include <Arduino.h>
#include <ctype.h>

//
// SerialConsole<32> console(processCommand);
// void loop() {
// console.handleInput();
// }
template <size_t INPUT_BUFFER_SIZE = 32> class SerialConsole {
    public:
        using CommandHandler = void (*)(const char*);

        explicit SerialConsole(CommandHandler handler) : handler_(handler) {}

        void handleInput() {
            bool updated = false;

            while (Serial.available()) {
                char c = static_cast<char>(Serial.read());

                if (c == '\r' || c == '\n') {
                    Serial.println();
                    if (input_length_ > 0 && handler_) {
                        // ここで
                        handler_(input_buffer_);
                        input_length_    = 0;
                        input_buffer_[0] = '\0';
                    }
                    updated = true;
                } else if (c == 0x08 || c == 0x7F) { // BS, DEL
                    if (input_length_ > 0) {
                        input_buffer_[--input_length_] = '\0';
                        updated                        = true;
                    }
                } else if (isprint(static_cast<unsigned char>(c))) {
                    if (input_length_ < INPUT_BUFFER_SIZE - 1) {
                        input_buffer_[input_length_++] = c;
                        input_buffer_[input_length_]   = '\0';
                        updated                        = true;
                    }
                }
            }

            if (updated) {
                refreshPrompt();
            }
        }

        void clearCurrentLine() {
            Serial.print('\r');
            Serial.print("                                                                                ");
            Serial.print('\r');
        }

        void refreshPrompt() {
            Serial.print('\r');
            Serial.print("> ");
            Serial.print(input_buffer_);

            const size_t now_len = 2 + input_length_; // "> " + 入力文字列
            if (last_prompt_len_ > now_len) {
                for (size_t i = 0; i < (last_prompt_len_ - now_len); i++) {
                    Serial.print(' ');
                }
                Serial.print('\r');
                Serial.print("> ");
                Serial.print(input_buffer_);
            }
            last_prompt_len_ = now_len;
        }

    private:
        CommandHandler handler_                         = nullptr;
        char           input_buffer_[INPUT_BUFFER_SIZE] = {0};
        size_t         input_length_                    = 0;
        size_t         last_prompt_len_                 = 0;
};