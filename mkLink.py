import os
import sys
import shutil

files = \
    (
        ("src/stm32Info.cpp", "../../../EclipseCPP/LatheCPP/lathe_src/stm32Info.cpp"),
    )

path = os.path.dirname(os.path.realpath(__file__))

makeLink = True
if len(sys.argv) >= 2:
    if sys.argv[1] == "-r":
        makeLink = False

def fWrite(f0, txt):
    f0.write(txt.encode())

# f = open("mklinkx.bat", "wb")
# fWrite(f, "@echo off\n\n")

for (link, srcFile) in files:
    # loc, link = link.split("/")
    # currentDir = os.getcwd()
    # os.chdir(loc)
    # if os.path.islink(link):
    #     print("path exists", link)
    #     os.unlink(link)

    # if makeLink:
    #     os.symlink(srcFile, link)
    # os.chdir(currentDir)

#     fWrite(f, "if exist \"%s\" (rm \"%s\")\n" \
#            "mklink \"%s\" \"%s\"\n\n" % (link, link, link, srcFile))
# f.close()

    copy = False
    lPath = os.path.join(path, os.sep.join(link.split('/')))
    sPath = os.path.join(path, os.sep.join(srcFile.split('/')[1:]))
    # print(lPath, sPath)
    if os.path.exists(lPath):
        # lTime = os.path.getmtime(lPath)
        # sTime = os.path.getmtime(sPath)
        lStat = os.stat(lPath)
        sStat = os.stat(sPath)
        # print(lPath, sPath)
        print("%-20s %5d  %5d %12.0f %12.0f" %
              (link, lStat.st_size, sStat.st_size, lStat.st_mtime, sStat.st_mtime))
        # print("lTime %d sTime %d" % (lTime, sTime))
        if sStat.st_size != lStat.st_size or sStat.st_mtime > lStat.st_mtime:
            copy = True
    else:
        copy = True
    if copy:
        print("copy", srcFile)
        shutil.copyfile(sPath, lPath)

    
