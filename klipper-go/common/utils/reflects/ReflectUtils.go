package reflects

import (
	"encoding/json"
	"fmt"
	"k3c/common/utils/str"
	"unsafe"

	"reflect"
)

// 生成新的对象
func ReflectNew(t reflect.Type) interface{} {
	if t.Kind() == reflect.Ptr {
		t = t.Elem()
	}
	if t.Kind() != reflect.Struct {
		return nil
	}
	obj := reflect.New(t) //
	return obj.Interface()
}

func GetPrivateFieldValue(obj interface{}, fieldName string) interface{} {
	if obj == nil {
		return nil
	}
	reflectValue := reflect.ValueOf(obj)

	if reflectValue.Kind() == reflect.Ptr {
		reflectValue = reflectValue.Elem()
	}
	value := reflectValue.FieldByName(fieldName)
	engine := reflect.NewAt(value.Type(), unsafe.Pointer(value.UnsafeAddr())).Elem()
	return engine.Interface()
}

// 根据字段名称取值
func ReflectFieldValue(obj interface{}, fieldName string) interface{} {
	reflectType := reflect.TypeOf(obj)
	if reflectType.Kind() == reflect.Ptr {
		reflectType = reflectType.Elem()
	}
	_, ok := reflectType.FieldByName(fieldName)
	if ok {
		reflectValue := reflect.ValueOf(obj)
		if reflectValue.Kind() == reflect.Ptr {
			reflectValue = reflectValue.Elem()
		}
		value := reflectValue.FieldByName(fieldName)
		return value.Interface()
	}
	return nil
}

// 根据方法名称运行
func ReflectMethod(obj interface{}, methodName string, paramMap map[string]interface{}) []interface{} {
	reflectType := reflect.TypeOf(obj)
	if reflectType.Kind() == reflect.Ptr {
		reflectType = reflectType.Elem()
	}
	if reflectType.Kind() != reflect.Struct {
		return nil
	}
	reflectValue := reflect.Indirect(reflect.ValueOf(obj))
	method := reflectValue.MethodByName(methodName)
	if method.Kind() == reflect.Func {
		var args = reqArgs(method, paramMap)
		if method.Type().NumIn() != len(args) {
			//illage param
			resJsonStr := "{\"code\":\"518\",\"error\":\"请求参数数量不对\"}"
			return []interface{}{resJsonStr}
		} else {
			return resCall(method, args)
		}
	}
	resJsonStr := "{\"code\":\"519\",\"error\":\"服务不存在或是未授权\"}"
	return []interface{}{resJsonStr}
}

func Hasattr(obj interface{}, methodName string) bool {
	reflectType := reflect.TypeOf(obj)
	//if reflectType.Kind() == reflect.Ptr {
	//	reflectType = reflectType.Elem()
	//}
	_, ok := reflectType.MethodByName(methodName)

	return ok
}

// 取得可运行方法对象:是inerface
func GetMethod(obj interface{}, methodName string) interface{} {
	reflectType := reflect.TypeOf(obj)
	if reflectType.Kind() == reflect.Ptr {
		reflectType = reflectType.Elem()
	}
	if reflectType.Kind() != reflect.Struct {
		return nil
	}
	reflectValue := reflect.ValueOf(obj)
	method := reflectValue.MethodByName(methodName)
	if method.Kind() == reflect.Func {
		return method
	}
	method = reflectValue.Elem().FieldByName(methodName)
	if method.Kind() == reflect.Func {
		return method
	}

	return nil
}
func ReqArgs(method reflect.Value, paramMap map[string]interface{}) interface{} {
	var args = make([]reflect.Value, 0)
	if paramMap != nil {
		numIn := method.Type().NumIn()
		if numIn > 0 {
			for _, v := range paramMap {
				args = append(args, reflect.ValueOf(v))
			}
			if len(args) > numIn {
				args = args[0:numIn]
			}
		}
	}
	ret := method.Call(args)
	if len(ret) != 0 {
		return method.Call(args)[0].Interface()
	}
	return nil
}
func reqArgs(method reflect.Value, paramMap map[string]interface{}) []reflect.Value {
	var args = make([]reflect.Value, 0)
	if paramMap != nil {
		numIn := method.Type().NumIn()
		if numIn > 0 {
			typeStr := method.Type().In(0).Name()
			if typeStr != "string" {
				return reqArgs4Do(method.Type().In(0), paramMap)
			} else {
				for _, v := range paramMap {
					args = append(args, reflect.Indirect(reflect.ValueOf(v)))
				}
				if len(args) > numIn {
					args = args[0:numIn]
				}
			}
		}
	}
	return args
}
func reqArgs4Do(t reflect.Type, paramMap map[string]interface{}) []reflect.Value {
	var args = make([]reflect.Value, 0)
	if paramMap != nil {
		bs, err := json.Marshal(paramMap)
		if err != nil {
			fmt.Println(err.Error())
			return args
		}
		do := ReflectNew(t)
		err = json.Unmarshal(bs, do)
		if err != nil {
			fmt.Println(err.Error())
			return args
		}

		args = append(args, reflect.Indirect(reflect.ValueOf(do)))

	} else {
		v := reflect.New(t) //
		args = append(args, v)
	}
	return args
}

func resCall(method reflect.Value, args []reflect.Value) []interface{} {

	numOut := method.Type().NumOut()
	if numOut > 0 {
		typeStr := method.Type().Out(0).Name()
		if typeStr != "string" {
			return resCal4Dto(method, args)
		} else {
			res := method.Call(args)
			size := len(res)
			if size > 0 {
				//log.Println(res[0].Type(), res[1].Type())

				ret := make([]interface{}, size)
				for i := 0; i < size; i++ {
					ret[i] = res[i].Interface()
				}
				return ret
			} else {
				fmt.Println("error")
				resJsonStr := "{\"code\":\"516\",\"error\":\"没有返回值\"}"
				return []interface{}{resJsonStr}
			}
		}
	} else {
		//no out return
		method.Call(args)
		resJsonStr := "{\"code\":\"200\",\"error\":\"\"}"
		return []interface{}{resJsonStr}
	}

	//if len(res) > 0 {
	//	//log.Println(res[0].Type(), res[1].Type())
	//	size := len(res)
	//	ret := make([]interface{}, size)
	//	for i := 0; i < size; i++ {
	//		ret[i] = res[i].Interface()
	//	}
	//	return ret
	//}
	//return nil
}
func resCal4Dto(method reflect.Value, args []reflect.Value) []interface{} {
	res := method.Call(args)
	size := len(res)
	if size > 0 {
		dto := res[0].Interface()
		bs, err := json.Marshal(dto)
		if err != nil {
			fmt.Println(err.Error())
			resJsonStr := "{\"code\":\"515\",\"error\":\"dto转换失败\"}"
			return []interface{}{resJsonStr}
		}
		resJsonStr := str.Bytes2String(bs)
		return []interface{}{resJsonStr}
	} else {
		fmt.Println("error")
		resJsonStr := "{\"code\":\"516\",\"error\":\"没有返回值\"}"
		return []interface{}{resJsonStr}
	}
}

//func CallByMethodName( obj interface{} ,methodName string,params []interface{}) ( []interface{} ){
//	//获得结构体的类型
//	reflectType := reflect.TypeOf(obj)
//	if reflectType.Kind() == reflect.Ptr {
//		reflectType=reflectType.Elem()
//	}
//	if reflectType.Kind() != reflect.Struct {
//		return nil
//	}
//	reflectValue := reflect.ValueOf(obj)
//	if reflectValue.Kind() == reflect.Ptr {
//		reflectValue=reflectValue.Elem()
//	}
//	value:= reflectValue.FieldByName(name)
//	if value.Kind() == reflect.Ptr {
//		value=value.Elem()
//	}
//	if value.Kind() != reflect.Struct {
//		return nil
//	}
//	fmt.Println(value.String(),value.Interface(),value.NumField(),value.NumMethod())
//	method:=value.MethodByName(methodName)
//	fmt.Println(method.Kind().String(),method.String())
//	if method.Kind() == reflect.Func {
//		args := []reflect.Value{reflect.ValueOf("1"),reflect.ValueOf("2"),reflect.ValueOf("3")}
//		method.Call(args)
//	}
//	return nil
//}

func IsNil(i interface{}) bool {
	if i == nil {
		return true
	}
	vi := reflect.ValueOf(i)
	return vi.IsNil()
}
