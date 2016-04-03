
用 SmartRF Flash Programmer 烧录 生成hex文件 提示 “HEX file content at address 0xXXXX exceeds chip's 256 kB flash size” ？


解决方法：

f8w2530.xcl文件，并打开。（这个文件在 “Projects\zstack\Tools\CC2530DB\”目录下，也可以通过IAR编译环境的左侧Workspace

窗口点开Tools文件夹看到）

在f8w2530.xcl文件中找到两行被注

释掉的语句：

           //-M(CODE)[(_CODEBANK_START+_FIRST_BANK_ADDR)-(_CODEBANK_END

+_FIRST_BANK_ADDR)]*\ 

         //_NR_OF_BANKS+_FIRST_BANK_ADDR=0x8000           

把这两行前面的“//”去掉，保存，重新编译，OK！