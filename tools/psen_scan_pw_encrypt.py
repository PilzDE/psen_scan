#!/usr/bin/python

original_pw = raw_input("Enter decrypted password: ")
encrypted_pw = ''

for i in range(0, len(original_pw)):
    encrypted_char = format(((ord(original_pw[i]) ^ 0xCD) + 100 * i) % 256 , '02x')
    encrypted_pw += encrypted_char

print encrypted_pw
