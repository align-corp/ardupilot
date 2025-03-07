#!/usr/bin/env python

'''
script to create ap_romfs_embedded.h from a set of static files

Andrew Tridgell
May 2017
'''

import os, sys, tempfile, gzip

def write_encode(out, s):
    out.write(s.encode())

def embed_file(out, f, idx, embedded_name, uncompressed):
    '''embed one file'''
    try:
        contents = open(f,'rb').read()
    except Exception:
        raise Exception("Failed to embed %s" % f)

    pad = 0
    if embedded_name.endswith("bootloader.bin"):
        # round size to a multiple of 32 bytes for bootloader, this ensures
        # it can be flashed on a STM32H7 chip
        blen = len(contents)
        pad = (32 - (blen % 32)) % 32
        if pad != 0:
            if sys.version_info[0] >= 3:
                contents += bytes([0xff]*pad)
            else:
                for i in range(pad):
                    contents += bytes(chr(0xff))
            print("Padded %u bytes for %s to %u" % (pad, embedded_name, len(contents)))

    crc = crc32(bytearray(contents))
    write_encode(out, '__EXTFLASHFUNC__ static const uint8_t ap_romfs_%u[] = {' % idx)

    compressed = tempfile.NamedTemporaryFile()
    if uncompressed:
        # ensure nul termination
        if sys.version_info[0] >= 3:
            nul = bytearray(0)
        else:
            nul = chr(0)
        if contents[-1] != nul:
            contents += nul
        compressed.write(contents)
    else:
        # compress it
        f = open(compressed.name, "wb")
        with gzip.GzipFile(fileobj=f, mode='wb', filename='', compresslevel=9, mtime=0) as g:
            g.write(contents)
        f.close()

    compressed.seek(0)
    b = bytearray(compressed.read())
    compressed.close()
    
    for c in b:
        write_encode(out, '%u,' % c)
    write_encode(out, '};\n\n');
    return crc

def crc32(bytes, crc=0):
    '''crc32 equivalent to crc32_small() from AP_Math/crc.cpp'''
    for byte in bytes:
        crc ^= byte
        for i in range(8):
            mask = (-(crc & 1)) & 0xFFFFFFFF
            crc >>= 1
            crc ^= (0xEDB88320 & mask)
    return crc

def create_embedded_h(filename, files, uncompressed=False):
    '''create a ap_romfs_embedded.h file'''

    out = open(filename, "wb")
    write_encode(out, '''// generated embedded files for AP_ROMFS\n\n''')

    # remove duplicates and sort
    files = sorted(list(set(f for f in files if not f[0].endswith(".DS_Store"))))
    crc = {}
    for i in range(len(files)):
        (name, filename) = files[i]
        try:
            crc[filename] = embed_file(out, filename, i, name, uncompressed)
        except Exception as e:
            print(e)
            return False

    write_encode(out, '''const AP_ROMFS::embedded_file AP_ROMFS::files[] = {\n''')

    for i in range(len(files)):
        (name, filename) = files[i]
        if uncompressed:
            ustr = ' (uncompressed)'
        else:
            ustr = ''
        print("Embedding file %s:%s%s" % (name, filename, ustr))
        write_encode(out, '{ "%s", sizeof(ap_romfs_%u), 0x%08x, ap_romfs_%u },\n' % (name, i, crc[filename], i))
    write_encode(out, '};\n')
    out.close()
    return True

if __name__ == '__main__':
    import sys
    flist = []
    for i in range(1, len(sys.argv)):
        f = sys.argv[i]
        flist.append((f, f))
    create_embedded_h("/tmp/ap_romfs_embedded.h", flist)
