### 1、Set up Go proxy
```
set GOPROXY=https://goproxy.bj.bcebos.com/ 
set GOPROXY=https://mirrors.aliyun.com/goproxy/
```
### 2、Project Construction Approach
#### 2.1、32-bit linux system
```
make tidy
make build
```
#### 2.2、32-bit arm linux system
```
make tidy
make build-arm
```
#### 2.3、clean prject
```
make clean
```