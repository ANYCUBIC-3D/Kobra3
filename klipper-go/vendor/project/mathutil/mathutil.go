/*
# Simple math helper functions
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

*/
package mathutil

import (
	"errors"
	"fmt"
	"k3c/common/value"
	"k3c/project"
	"log"
	"math"
	"sync"
)

/*
######################################################################
# Coordinate descent
######################################################################

*/
// Helper code that implements coordinate descent
func Coordinate_descent(adj_params []string, params map[string]float64, error_func func(map[string]float64) float64) map[string]float64 {
	// Define potential changes
	dp := map[string]float64{}
	for _, param_name := range adj_params {
		dp[param_name] = 1.
	}
	// Calculate the error
	best_err := error_func(params)
	log.Printf("Coordinate descent initial error: %s", best_err)

	threshold := 0.00001
	rounds := 0

	for sum(values(dp)) > threshold && rounds < 10000 {
		rounds += 1
		for _, param_name := range adj_params {
			orig := params[param_name]
			params[param_name] = orig + dp[param_name]
			err := error_func(params)
			if err < best_err {
				// There was some improvement
				best_err = err
				dp[param_name] *= 1.1
				continue
			}
			params[param_name] = orig - dp[param_name]
			err = error_func(params)
			if err < best_err {
				// There was some improvement
				best_err = err
				dp[param_name] *= 1.1
				continue
			}
			params[param_name] = orig
			dp[param_name] *= 0.9
		}
	}
	log.Printf("Coordinate descent best_err: %s  rounds: %d",
		best_err, rounds)
	return params

}

func sum(values []float64) float64 {
	sum_val := 0.
	for _, val := range values {
		sum_val += val
	}
	return sum_val
}

func values(dp map[string]float64) []float64 {
	values_arr := []float64{}
	for _, val := range dp {
		values_arr = append(values_arr, val)
	}
	return values_arr
}

/*
# Helper to run the coordinate descent function in a background
# process so that it does not block the main thread.
*/
type msg_node struct {
	f   bool
	res interface{}
}

func Background_coordinate_descent(printer project.Printer, adj_params []string, params map[string]float64, error_func func(map[string]float64) float64) (interface{}, project.error) {
	// parent_conn, child_conn = multiprocessing.Pipe()
	cha := make(chan msg_node)
	defer close(cha)
	counter := 0
	lock := &sync.Mutex{}
	wrapper := func(lock *sync.Mutex, counter1 *int) {
		lock.Lock()
		defer func() {
			*counter1++
			lock.Unlock()
		}()
		queuelogger.clear_bg_logging()
		res := Coordinate_descent(adj_params, params, error_func)
		if err := recover(); err != nil {
			cha <- msg_node{true, err}
			return
		}
		cha <- msg_node{false, res}

	}
	// Start a process to perform the calculation
	go wrapper(lock, &counter)
	// Wait for the process to finish
	reactor := printer.Get_reactor()
	gcode, err := printer.Lookup_object("gcode", nil)
	if err != nil {
		value.StaticValue.Error.Println(err)
	}
	eventtime := reactor.Monotonic()
	last_report_time := eventtime
	for {
		lock.Lock()
		c := counter
		lock.Unlock()
		if c > 0 {
			break
		}
		if eventtime > last_report_time+5 {
			last_report_time = eventtime
			dispatch := gcode.(project.GCodeDispatch)
			dispatch.Respond_info("Working on calibration...", false)
		}
		eventtime = reactor.Pause(eventtime + .1)
	}
	// Return results
	resData := <-cha
	is_err := resData.f
	res := resData.res
	if is_err {
		return nil, errors.New(fmt.Sprintf("Error in coordinate descent: %s", res))
	}
	return res, nil

}

/*
######################################################################
# Trilateration
######################################################################

# Trilateration finds the intersection of three spheres.  See the
# wikipedia article for the details of the algorithm.
*/
func Trilateration(sphere_coords [][]float64, radius2 []float64) []float64 {
	sphere_coord1 := sphere_coords[0]
	sphere_coord2 := sphere_coords[1]
	sphere_coord3 := sphere_coords[2]
	s21 := Matrix_sub(sphere_coord2, sphere_coord1)
	s31 := Matrix_sub(sphere_coord3, sphere_coord1)

	d := math.Sqrt(Matrix_magsq(s21))
	ex := Matrix_mul(s21, 1./d)
	i := Matrix_dot(ex, s31)
	vect_ey := Matrix_sub(s31, Matrix_mul(ex, i))
	ey := Matrix_mul(vect_ey, 1./math.Sqrt(Matrix_magsq(vect_ey)))
	ez := Matrix_cross(ex, ey)
	j := Matrix_dot(ey, s31)

	x := (radius2[0] - radius2[1] + math.Pow(d, 2)) / (2. * d)
	y := (radius2[0] - radius2[2] - math.Pow(x, 2) + math.Pow((x-i), 2) + math.Pow(j, 2)) / (2. * j)
	z := -math.Sqrt(radius2[0] - math.Pow(x, 2) - math.Pow(y, 2))

	ex_x := Matrix_mul(ex, x)
	ey_y := Matrix_mul(ey, y)
	ez_z := Matrix_mul(ez, z)

	return Matrix_add(sphere_coord1, Matrix_add(ex_x, Matrix_add(ey_y, ez_z)))
}

/*
######################################################################
# Matrix helper functions for 3x1 matrices
######################################################################
*/
func Matrix_cross(m1, m2 []float64) []float64 {
	return []float64{
		m1[1]*m2[2] - m1[2]*m2[1],
		m1[2]*m2[0] - m1[0]*m2[2],
		m1[0]*m2[1] - m1[1]*m2[0]}
}
func Matrix_dot(m1, m2 []float64) float64 {
	return m1[0]*m2[0] + m1[1]*m2[1] + m1[2]*m2[2]
}

func Matrix_magsq(m1 []float64) float64 {
	return math.Pow(m1[0], 2) + math.Pow(m1[1], 2) + math.Pow(m1[2], 2)
}

func Matrix_add(m1, m2 []float64) []float64 {
	return []float64{m1[0] + m2[0], m1[1] + m2[1], m1[2] + m2[2]}
}

func Matrix_sub(m1, m2 []float64) []float64 {
	return []float64{m1[0] - m2[0], m1[1] - m2[1], m1[2] - m2[2]}
}

func Matrix_mul(m1 []float64, s float64) []float64 {
	return []float64{m1[0] * s, m1[1] * s, m1[2] * s}
}
