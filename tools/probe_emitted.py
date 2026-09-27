from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

enc = open(r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2\second_loader.enc', 'rb').read()
print('enc[0x2C0:0x2E0] =', enc[0x2C0:0x2E0].hex(' '))
target = bytes.fromhex('9520c36f0da66bc18d1adc02eb34ec1d')
src = enc[0x2C0:0x2C0 + 16]

CAND = {
    'aa/af5f': (bytes([0xAA] * 16), bytes.fromhex('AF5F2CB04AC1751ABF51CEF1C8096210')),
    'aa/zero': (bytes([0xAA] * 16), bytes(16)),
    'zero/zero': (bytes(16), bytes(16)),
    'aa/enpE0': (bytes([0xAA] * 16), enc[0xE0:0xF0]),
    'encE0/encF0': (enc[0xE0:0xF0], enc[0xF0:0x100]),
    'encE0/af5f': (enc[0xE0:0xF0], bytes.fromhex('AF5F2CB04AC1751ABF51CEF1C8096210')),
    'encE0/zero': (enc[0xE0:0xF0], bytes(16)),
    'enc100/enc110': (enc[0x100:0x110], enc[0x110:0x120]),
}
for name, (k, iv) in CAND.items():
    out = Cipher(algorithms.AES(k), modes.CBC(iv)).decryptor().update(src)
    mark = '   <== MATCHES EMULATOR' if out == target else ''
    print('%-16s %s%s' % (name, out.hex(' '), mark))
