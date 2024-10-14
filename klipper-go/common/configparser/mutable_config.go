package configparser

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"hash/adler32"
	"io"
	"k3c/common/utils/cast"
	"k3c/common/utils/file"
	"log"
	"os"
	"path/filepath"
	"strings"
	"time"
)

const (
	MagicNumber = "K3C"
)

type EncryptType int

const (
	EncryptNone EncryptType = iota
	EncryptAes
)

const (
	MutableConfigFileName = "printer_mutable.cfg"
)

type MutableConfig struct {
	file            string
	sections        Sections
	originEncrypt   EncryptType
	originPlainText bool
	encrypt         EncryptType
	plaintext       bool
}

type MutableConfigInfo struct {
	Filename string      `json:"fname"`
	Length   int         `json:"len"`
	Version  string      `json:"ver"`
	Encrypt  EncryptType `json:"encrypt"`
	CheckSum bool        `json:"checksum"`
	Salt     string      `json:"salt"`
}

type Sections map[string]Section
type Section map[string]interface{}

func NewMutableConfig(filename string, encryptType EncryptType, plaintext bool) (*MutableConfig, error) {
	_, err := os.Stat(filename)
	if err != nil {
		if os.IsNotExist(err) {
			config := &MutableConfig{
				file:            filename,
				sections:        make(Sections),
				encrypt:         encryptType,
				plaintext:       plaintext,
				originEncrypt:   encryptType,
				originPlainText: plaintext,
			}
			return config, nil
		}
		return nil, err
	}

	return loadBinaryConfig(filename, encryptType, plaintext)
}

func loadPlaintextConfig(filename string, encryptType EncryptType, plaintext bool) (*MutableConfig, error) {
	contentBytes, err := os.ReadFile(filename)
	if len(contentBytes) > len(MagicNumber) {
		if bytes.Equal(contentBytes[:len(MagicNumber)], []byte(MagicNumber)) {
			return loadBinaryConfig(filename, encryptType, plaintext)
		}
	}
	var sections Sections
	if err = json.Unmarshal(contentBytes, &sections); err != nil {
		return nil, err
	}
	config := &MutableConfig{
		file:            filename,
		sections:        sections,
		encrypt:         encryptType,
		plaintext:       plaintext,
		originEncrypt:   EncryptNone,
		originPlainText: true,
	}
	return config, nil
}

func loadBinaryConfig(filename string, encryptType EncryptType, plaintext bool) (*MutableConfig, error) {
	f, err := os.Open(filename)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	bufr := bufio.NewReader(f)
	magicNumber := make([]byte, len(MagicNumber))
	_, err = io.ReadFull(bufr, magicNumber)
	if magicNumber[0] == '{' {
		return loadPlaintextConfig(filename, encryptType, plaintext)
	}

	if !bytes.Equal(magicNumber, []byte(MagicNumber)) {
		return nil, fmt.Errorf("config invalid magic number: %v", string(magicNumber))
	}

	var infoLen uint32
	infoLenBytes := make([]byte, 4)
	_, err = io.ReadFull(bufr, infoLenBytes)
	if err != nil {
		return nil, fmt.Errorf("config read info lenght error: %v", err)
	}

	infoLen = binary.BigEndian.Uint32(infoLenBytes)
	infoBytes := make([]byte, infoLen)
	_, err = io.ReadFull(bufr, infoBytes)
	if err != nil {
		return nil, fmt.Errorf("config read info error: %v", err)
	}

	var info MutableConfigInfo
	if err = json.Unmarshal(infoBytes, &info); err != nil {
		return nil, err
	}

	contentBytes := make([]byte, info.Length)
	_, err = io.ReadFull(bufr, contentBytes)
	if err != nil {
		return nil, fmt.Errorf("config read content error: %v", err)
	}

	if info.Encrypt != EncryptNone {
		return nil, errors.New("config not support encrypt")
	}

	if info.CheckSum {
		checksumBytes := make([]byte, 4)
		_, err = io.ReadFull(bufr, checksumBytes)
		if err != nil {
			return nil, fmt.Errorf("config read checksum error: %v", err)
		}

		checksum1 := binary.BigEndian.Uint32(checksumBytes[:])
		checksum2 := adler32.Checksum(contentBytes)
		if checksum1 != checksum2 {
			return nil, errors.New("invalid checksum")
		}
	}

	var sections Sections
	if err = json.Unmarshal(contentBytes, &sections); err != nil {
		return nil, fmt.Errorf("config content unmarshal error: %v", err)
	}

	config := &MutableConfig{
		file:            filename,
		sections:        sections,
		encrypt:         encryptType,
		plaintext:       plaintext,
		originEncrypt:   info.Encrypt,
		originPlainText: false,
	}
	return config, err
}

func (c *MutableConfig) SaveAndBackup() (err error) {
	err1 := file.DeleteBackFileNameWhileMore(c.file, ".cfg", 3)
	if err1 != nil {
		log.Println(err1)
	}
	//
	path, _ := filepath.Abs(c.file)
	path = filepath.Dir(path)
	basename := filepath.Base(c.file)
	tempname := filepath.Join(path, basename+".temp")
	err = c.SaveAs(tempname)
	if err != nil {
		return err
	}

	if _, err = os.Stat(c.file); err == nil {
		segments := strings.Split(basename, ".")
		fileext := "cfg"
		if len(segments) > 1 {
			fileext = segments[1]
		}
		savename := fmt.Sprintf("%s_%s.%s", segments[0], time.Now().Format("20060102150405"), fileext)
		if err = os.Rename(c.file, filepath.Join(path, savename)); err != nil {
			return err
		}
	}
	return os.Rename(tempname, c.file)
}

func (c *MutableConfig) SaveBinaryFormatAs(filename string) error {
	bf := bytes.NewBuffer(nil)
	bf.Write([]byte(MagicNumber))

	var info MutableConfigInfo
	info.Filename = filename
	contentBytes, err := json.Marshal(c.sections)
	if err != nil {
		return err
	}

	info.Length = len(contentBytes)
	info.Encrypt = 0
	info.Version = "1.0"
	info.CheckSum = true

	infoLenBytes := make([]byte, 4)
	infoBytes, err := json.Marshal(info)
	if err != nil {
		return nil
	}
	binary.BigEndian.PutUint32(infoLenBytes, uint32(len(infoBytes)))
	bf.Write(infoLenBytes)
	bf.Write(infoBytes)

	bf.Write(contentBytes)

	checksum := adler32.Checksum(contentBytes)
	checksumBytes := make([]byte, 4)
	binary.BigEndian.PutUint32(checksumBytes[0:], checksum)
	bf.Write(checksumBytes)
	f, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer f.Close()
	_, err = bf.WriteTo(f)
	return err
}

func (c *MutableConfig) SaveAs(filename string) error {
	if c.plaintext {
		return c.SavePlainTextFormatAs(filename)
	}
	return c.SaveBinaryFormatAs(filename)
}

func (c *MutableConfig) SavePlainTextFormatAs(filename string) error {
	contentBytes, err := json.MarshalIndent(c.sections, "", "\t")
	if err != nil {
		return err
	}
	return os.WriteFile(filename, contentBytes, 0666)
}

func (c *MutableConfig) AddSectionIfNotExist(section string) Section {
	if !c.HasSection(section) {
		c.sections[section] = make(Section)
	}
	return c.sections[section]
}

func (c *MutableConfig) Set(section, name string, val interface{}) {
	_, ok := c.sections[section]
	if !ok {
		c.sections[section] = make(Section)
	}
	c.sections[section][name] = val
}

func (c *MutableConfig) Section(section string) (Section, bool) {
	sec, ok := c.sections[section]
	return sec, ok
}

func (c *MutableConfig) HasSection(section string) bool {
	_, ok := c.sections[section]
	return ok
}

func (c *MutableConfig) ToInIString() string {
	var builder strings.Builder
	for section, optionvals := range c.sections {
		builder.WriteString("[" + section + "]\n")

		for opt, val := range optionvals {
			valstr := cast.ToString(val)
			if strings.Index(valstr, "\n") != -1 {
				builder.WriteString(opt + " = ")
				vals := strings.Split(valstr, "\n")
				for _, v := range vals {
					builder.WriteString("\n     " + v)
				}
			} else {
				builder.WriteString(opt + " = " + valstr + "\n")
			}
			builder.WriteString("\n")
		}
	}

	return builder.String()
}

func (s Section) Get(name string) interface{} {
	return s[name]
}

func (s Section) Has(name string) bool {
	_, ok := s[name]
	return ok
}

func (s Section) Set(name string, val interface{}) {
	s[name] = val
}
