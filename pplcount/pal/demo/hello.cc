// hello.cc
#include <node.h>
#include <string>
#include <ctime>
#include <cstdlib>
#include <iostream>
namespace demo {


using v8::Exception;
using v8::FunctionCallbackInfo;
using v8::Isolate;
using v8::Local;
using v8::Number;
using v8::Object;
using v8::String;
using v8::Value;
int prod;
int inc=0;
void Method(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
	
			srand(time(NULL));
			prod=rand()%1000+1;
			 Local<Number> num = Number::New(isolate, prod);
			//std::string s = std::to_string(prod);
			//args.GetReturnValue().Set(String::NewFromUtf8(isolate, prod));
			args.GetReturnValue().Set(num);
  
}

void init(Local<Object> exports) {

  NODE_SET_METHOD(exports, "hello", Method);
}

NODE_MODULE(addon, init)

}  // namespace demo

