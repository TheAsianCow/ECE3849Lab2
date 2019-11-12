# invoke SourceDir generated makefile for rtos.pem4f
rtos.pem4f: .libraries,rtos.pem4f
.libraries,rtos.pem4f: package/cfg/rtos_pem4f.xdl
	$(MAKE) -f C:\Users\JEFFRE~1\DOCUME~1\WPI\2019-2~1\BTERM~1\ECE3849\labs\ece3849_lab2_jyhuang_rskirschner/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\JEFFRE~1\DOCUME~1\WPI\2019-2~1\BTERM~1\ECE3849\labs\ece3849_lab2_jyhuang_rskirschner/src/makefile.libs clean

