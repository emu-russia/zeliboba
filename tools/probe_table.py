import hashlib
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

enc = open(r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2\second_loader.enc', 'rb').read()
enp = open(r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2\second_loader.enp', 'rb').read()
bin_ = open(r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2_dec\second_loader.bin', 'rb').read()

KEY = bytes([0xAA] * 16)
IV = bytes.fromhex('AF5F2CB04AC1751ABF51CEF1C8096210')

print('sha256(enc payload) =', hashlib.sha256(enc[0x2C0:0x2C0 + 0x16600]).hexdigest())
print('sha256(dec payload) =', hashlib.sha256(bin_).hexdigest())

for name, src in (('enc', enc), ('enp', enp)):
    for off in (0x100, 0x120, 0x140, 0x160, 0x180, 0x1A0):
        blk = src[off:off + 32]
        for mode, iv in (('ecb', bytes(16)), ('cbc', IV)):
            c = Cipher(algorithms.AES(KEY), modes.ECB() if mode == 'ecb' else modes.CBC(iv)).decryptor()
            out = c.update(blk) + c.finalize()
            tag = ''
            if out.hex() == hashlib.sha256(enc[0x2C0:0x2C0 + 0x16600]).hexdigest():
                tag += ' == sha256(enc payload)'
            if out.hex() == hashlib.sha256(bin_).hexdigest():
                tag += ' == sha256(dec payload)'
            print('%s +%03X %s -> %s%s' % (name, off, mode, out.hex(), tag))
