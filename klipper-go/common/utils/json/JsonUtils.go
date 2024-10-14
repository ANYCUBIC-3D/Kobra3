package jsons

import (
	"encoding/json"
	"k3c/common/jsons/model"
)

func GetBytes(model *model.Config) ([]byte, error) {
	output, err := json.Marshal(&model)
	return output, err
}
func MapKey2Slice(res map[interface{}]interface{}) []interface{} {
	t := []interface{}{}
	for k, _ := range res {
		t = append(t, k)
	}
	return t
}
func MapKey2Slice1(res map[string]interface{}) []string {
	t := []string{}
	for k, _ := range res {
		t = append(t, k)
	}
	return t
}
