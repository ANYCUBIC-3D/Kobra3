package maths

import "math"

// polyfit 函数执行二次多项式拟合
func Polyfit(x, y []float64, degree int) ([]float64, [2]float64) {
	n := len(x)
	xMean := mean(x)
	xStdDev := stdDev(x, xMean)

	// 标准化 x
	xNormalized := make([]float64, n)
	for i := range x {
		xNormalized[i] = (x[i] - xMean) / xStdDev
	}

	// 构建矩阵和向量
	X := make([][]float64, n)
	for i := range X {
		X[i] = make([]float64, degree+1)
		for j := 0; j <= degree; j++ {
			X[i][j] = math.Pow(xNormalized[i], float64(degree-j))
		}
	}

	// 矩阵转置
	XT := _transpose(X)

	// XT * X
	XTX := matMul(XT, X)

	// XT * y
	XTy := make([]float64, degree+1)
	for i := range XTy {
		for j := 0; j < n; j++ {
			XTy[i] += XT[i][j] * y[j]
		}
	}

	// 解方程 XTX * p = XTy
	p := gaussJordan(XTX, XTy)

	return p, [2]float64{xMean, xStdDev}
}

// polyval 函数计算多项式值
func Polyval(p []float64, x []float64, mu [2]float64) []float64 {
	y := make([]float64, len(x))
	for i := range x {
		xNormalized := (x[i] - mu[0]) / mu[1]
		y[i] = p[0]*xNormalized*xNormalized + p[1]*xNormalized + p[2]
	}
	return y
}

// mean 函数计算数据的均值
func mean(data []float64) float64 {
	sum := 0.0
	for _, v := range data {
		sum += v
	}
	return sum / float64(len(data))
}

// stdDev 函数计算数据的标准差
func stdDev(data []float64, mean float64) float64 {
	var sum float64
	for _, v := range data {
		sum += (v - mean) * (v - mean)
	}
	return math.Sqrt(sum / float64(len(data)))
}

// transpose 函数计算矩阵的转置
func _transpose(matrix [][]float64) [][]float64 {
	rows := len(matrix)
	cols := len(matrix[0])
	_transpose := make([][]float64, cols)
	for i := range _transpose {
		_transpose[i] = make([]float64, rows)
		for j := range matrix {
			_transpose[i][j] = matrix[j][i]
		}
	}
	return _transpose
}

// matMul 函数计算矩阵乘法
func matMul(a, b [][]float64) [][]float64 {
	rows := len(a)
	cols := len(b[0])
	mul := make([][]float64, rows)
	for i := range mul {
		mul[i] = make([]float64, cols)
		for j := range b[0] {
			for k := range b {
				mul[i][j] += a[i][k] * b[k][j]
			}
		}
	}
	return mul
}

// gaussJordan 函数解方程组
func gaussJordan(a [][]float64, b []float64) []float64 {
	n := len(b)
	augmented := make([][]float64, n)
	for i := range augmented {
		augmented[i] = make([]float64, n+1)
		copy(augmented[i], a[i])
		augmented[i][n] = b[i]
	}

	// 前向消元
	for i := 0; i < n; i++ {
		// 寻找主元
		maxRow := i
		for k := i + 1; k < n; k++ {
			if math.Abs(augmented[k][i]) > math.Abs(augmented[maxRow][i]) {
				maxRow = k
			}
		}
		augmented[i], augmented[maxRow] = augmented[maxRow], augmented[i]

		// 消元
		for k := i + 1; k < n; k++ {
			c := -augmented[k][i] / augmented[i][i]
			for j := i; j < n+1; j++ {
				if i == j {
					augmented[k][j] = 0
				} else {
					augmented[k][j] += c * augmented[i][j]
				}
			}
		}
	}

	// 回代
	x := make([]float64, n)
	for i := n - 1; i >= 0; i-- {
		x[i] = augmented[i][n] / augmented[i][i]
		for k := i - 1; k >= 0; k-- {
			augmented[k][n] -= augmented[k][i] * x[i]
		}
	}
	return x
}
