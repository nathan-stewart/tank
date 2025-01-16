#!/usr/bin/env python3


a = 51
b = 52

def mybinary(x):
    sval = ''
    while x > 0:
        sval = str(x % 2) + sval
        x = x // 2
    return '0b'+sval

print(a ^ b)
print(mybinary(a ^ b))
print(bin(a^b))
print(bin(a) == mybinary(a))