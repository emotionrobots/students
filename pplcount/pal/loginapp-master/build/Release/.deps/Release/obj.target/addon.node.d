cmd_Release/obj.target/addon.node := g++ -shared -pthread -rdynamic  -Wl,-soname=addon.node -o Release/obj.target/addon.node -Wl,--start-group Release/obj.target/addon/hello.o -Wl,--end-group 
