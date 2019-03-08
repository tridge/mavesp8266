import subprocess
import datetime

gitrevision = subprocess.check_output(["git", "rev-parse", "HEAD"]).strip()

currentDT = datetime.datetime.now()
builddate = currentDT.strftime("%Y-%m-%d")
buildtime = currentDT.strftime("%H:%M:%S")


print "-DPIO_SRC_REV=%s" % gitrevision,
print "-DPIO_BUILD_DATE=%s" % builddate,
print "-DPIO_BUILD_TIME=%s" % buildtime


