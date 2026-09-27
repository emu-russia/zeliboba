from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes

enc = open(r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2\second_loader.enc', 'rb').read()
bin_ = open(r'C:\Work\PSVita\Vita_104_Firmware\Out\SLB2_dec\second_loader.bin', 'rb').read()
rom = open(r'C:\Work\PSVita\dumps\vita_prototype_bootrom.bin', 'rb').read()
romb = lambda a, n: rom[a - 0x5C000:a - 0x5C000 + n]

print('rom5E704:', romb(0x5E704, 32).hex(' '))
print('rom5E724:', romb(0x5E724, 32).hex(' '))
# The 0x5C6E4 op: source = dest = image+0xE0, length 0x167E0, keyring 8.
start, length = 0xE0, 0x167E0
src = enc[start:start + length]
for kn, kmat in (('5E704', romb(0x5E704, 32)), ('5E724', romb(0x5E724, 32))):
    for ivname, iv in (('mat', kmat[16:32]), ('zero', bytes(16))):
        if len(iv) < 16:
            continue
        c = Cipher(algorithms.AES(kmat[:16]), modes.CBC(iv)).decryptor()
        out = c.update(src) + c.finalize()
        idx = out.find(bin_[:32])
        print('key=%s iv=%s: plaintext payload found at +0x%X' % (kn, ivname, idx if idx >= 0 else -1))
        if idx >= 0:
            print('   full match:', out[idx:idx + len(bin_)] == bin_)
