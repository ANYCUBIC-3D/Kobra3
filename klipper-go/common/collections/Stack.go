package collections

type Stack struct {
	slc []interface{}
}
func (s *Stack) Push(a interface{}) {
	s.slc = append(s.slc, a)
}
func (s *Stack) Pop() interface{} {
	a := s.slc[len(s.slc)-1]
	s.slc = s.slc[:len(s.slc)-1]
	return a
}
func (s *Stack) Len() interface{} {
	return len(s.slc)
}
