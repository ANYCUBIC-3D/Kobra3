### 1、设置Go代理
```
set GOPROXY=https://goproxy.bj.bcebos.com/ 
set GOPROXY=https://mirrors.aliyun.com/goproxy/
```
### 2、工程构建方法
#### 2.1、32位系统
```
make tidy
make build
```
#### 2.2、ARM版本
```
make tidy
make build-arm
```
#### 2.3、清除工程
```
make clean
```