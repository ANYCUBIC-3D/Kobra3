package maths

func Mean(arr [][]float64, axis interface{}) []float64 {
	var means []float64
	// 如果axis为nil，则计算所有元素的平均值
	if axis == nil {
		var sum float64
		var count int
		for _, row := range arr {
			for _, val := range row {
				sum += val
				count++
			}
		}

		means = append(means, sum/float64(count))
	} else {
		// 将axis转换为int类型
		var axisInt int
		switch axis.(type) {
		case int:
			axisInt = axis.(int)
		}

		// 计算指定轴的平均值
		if axisInt == 0 {
			// 计算每列的平均值
			for j := 0; j < len(arr[0]); j++ {
				var sum float64
				for i := 0; i < len(arr); i++ {
					sum += arr[i][j]
				}

				means = append(means, sum/float64(len(arr)))
			}
		} else if axisInt == 1 {
			// 计算每行的平均值
			for i := 0; i < len(arr); i++ {
				var sum float64

				for j := 0; j < len(arr[i]); j++ {
					sum += arr[i][j]
				}
				means = append(means, sum/float64(len(arr[i])))
			}
		}
	}
	return means
}

//func main() {
//	// Example 2D matrix
//	matrix := [][]float64{
//		{1, 2, 3},
//		{4, 5, 6},
//		{7, 8, 9},
//	}
//
//	// Calculate mean along axis 0
//	mean0 := Mean(matrix, 0)
//	fmt.Printf("Mean along axis 0: %v\n", mean0)
//
//	// Calculate mean along axis 1
//	mean1 := Mean(matrix, 1)
//	fmt.Printf("Mean along axis 1: %v\n", mean1)
//
//	// Calculate mean of flattened array
//	meanFlat := Mean(matrix, nil)
//	fmt.Printf("Mean of flattened array: %v\n", meanFlat)
//
//}
