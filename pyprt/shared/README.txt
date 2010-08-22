This folder contains the api.proto file which specifies the message format for sim <--> controller communication. After making any changes to api.proto, the language-specific implementation files will need to be recreated. To do this, run the following commands from this directory (assumes that the protobuffer compiler, protoc is installed and is on your system path:
protoc --cpp_out=. api.proto
protoc --python_out=. api.proto
protoc --java_out=. api.proto