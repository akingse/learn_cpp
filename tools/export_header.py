import os, re, sys, stat
if len(sys.argv) < 2: 
    exit()
strEncode = 'GB2312'
with open('pch.h', 'r', encoding=strEncode) as f:
    content = f.read()
    publicFiles = re.findall(r'#include\s*\"([0-9a-zA-Z\_\.]+)\"\s*//\s*export', content)
    stlList = re.findall(r'#include\s*\<([0-9a-zA-Z\_]+)\>', content)
def export_file(hfile, hfiles):
    if len(hfiles) == 0:
        return
    publicExhead = b''
    for stl in stlList:
        publicExhead += ('#include <{0}>\r\n'.format(stl)).encode(strEncode)
    for file in hfiles:
        with open(file, 'rb') as f:
            publicExhead += '''\r
/* Warning, change the file to script automatically.\r
Please modify the source file under source. \r
Source file: \"{0}\".*/'''.format(file).encode(strEncode)
            publicExhead += f.read() + b'\r\n'
    publicExhead = publicExhead.replace(b'#pragma once', b'')
    publicExhead = publicExhead.replace(b'__declspec(dllexport)', b'__declspec(dllimport)')
    publicExhead = b'#pragma once\r\n' + publicExhead + b'\r\n'
    os.makedirs('../auto_include/', exist_ok=True)
    if os.path.exists(hfile):
        os.chmod(hfile, stat.S_IWRITE)
    with open(hfile, 'wb') as f:
        f.write(publicExhead)
export_file('../auto_include/{0}_API.h'.format(sys.argv[1]), publicFiles)