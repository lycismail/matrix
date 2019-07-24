cd protocol\raw
protoc.exe -I.\ --cpp_out=..\ .\can.proto .\common.proto .\sensor.proto .\meta_data.proto .\meta.proto
pause