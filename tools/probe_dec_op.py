from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

enc = open(r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2\second_loader.enc', 'rb').read()
bin_ = open(r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2_dec\second_loader.bin', 'rb').read()
rom = open(r'C:\Work\PSVita\dumps\vita_prototype_bootrom.bin', 'rb').read()
romb = lambda a, n: rom[a - 0x5C000:a - 0x5C000 + n]

MAT = {
    'rom5E704': romb(0x5E704, 32),
    'rom5E724': romb(0x5E724, 32),
    'rom5E744': romb(0x5E744, 32),
    'rom5E764': romb(0x5E764, 32),
    'aa/af5f': bytes([0xAA] * 16) + bytes.fromhex('AF5F2CB04AC1751ABF51CEF1C8096210'),
    'enc+E0': enc[0xE0:0x100],
    'enc+100': enc[0x100:0x120],
}
L = len(bin_)
hits = []
for off in (0x2C0, 0x2A0, 0x2E0, 0xE0, 0x2C0 - 0x20, 0x2C0 + 0x20):
    if off + L > len(enc):
        continue
    src = enc[off:off + L]
    for mname, mat in MAT.items():
        for klen in (16, 32):
            key = mat[:klen]
            iv = mat[klen:klen + 16] if len(mat) >= klen + 16 else bytes(16)
            for ivname, ivv in (('mat', iv), ('zero', bytes(16)), ('af5f', bytes.fromhex('AF5F2CB04AC1751ABF51CEF1C8096210'))):
                c = Cipher(algorithms.AES(key), modes.CBC(ivv)).decryptor()
                out = c.update(src) + c.finalize()
                if out == bin_:
                    hits.append('off=0x%X key=%s-%d iv=%s' % (off, mname, klen * 8, ivname))
print('hits:', hits)
# Also: is the plaintext simply at some offset of enc (i.e. the payload is stored plaintext)?
for off in range(0, len(enc) - L + 1, 4):
    if enc[off:off + L] == bin_:
        print('plaintext copy at', hex(off))
print('bin[:16]', bin_[:16].hex(' '))
