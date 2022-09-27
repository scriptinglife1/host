// Copyright 2016 The Periph Authors. All rights reserved.
// Use of this source code is governed under the Apache License, Version 2.0
// that can be found in the LICENSE file.

// Package serial implements cross platform UART support exposed by the
// operating system.
//
// On POSIX, it is via devfs. On Windows, it is via Windows specific APIs.
package serial

import (
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"runtime"
	"sort"
	"strconv"
	"sync"
	"syscall"
	"unsafe"

	"periph.io/x/conn/v3"
	"periph.io/x/conn/v3/driver/driverreg"
	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/gpio/gpioreg"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/uart"
	"periph.io/x/conn/v3/uart/uartreg"
)

var baudrateMap = map[int]uint32{
	0:       syscall.B9600, // Default to 9600
	50:      syscall.B50,
	75:      syscall.B75,
	110:     syscall.B110,
	134:     syscall.B134,
	150:     syscall.B150,
	200:     syscall.B200,
	300:     syscall.B300,
	600:     syscall.B600,
	1200:    syscall.B1200,
	1800:    syscall.B1800,
	2400:    syscall.B2400,
	4800:    syscall.B4800,
	9600:    syscall.B9600,
	19200:   syscall.B19200,
	38400:   syscall.B38400,
	57600:   syscall.B57600,
	115200:  syscall.B115200,
	230400:  syscall.B230400,
	460800:  syscall.B460800,
	500000:  syscall.B500000,
	576000:  syscall.B576000,
	921600:  syscall.B921600,
	1000000: syscall.B1000000,
	1152000: syscall.B1152000,
	1500000: syscall.B1500000,
	2000000: syscall.B2000000,
	2500000: syscall.B2500000,
	3000000: syscall.B3000000,
	3500000: syscall.B3500000,
	4000000: syscall.B4000000,
}

var databitsMap = map[int]uint32{
	0: syscall.CS8, // Default to 8 bits
	5: syscall.CS5,
	6: syscall.CS6,
	7: syscall.CS7,
	8: syscall.CS8,
}

func setRawMode(settings *syscall.Termios) {
	// Set local mode
	settings.Cflag |= syscall.CREAD
	settings.Cflag |= syscall.CLOCAL

	// Set raw mode
	settings.Lflag &^= syscall.ICANON
	settings.Lflag &^= syscall.ECHO
	settings.Lflag &^= syscall.ECHOE
	settings.Lflag &^= syscall.ECHOK
	settings.Lflag &^= syscall.ECHONL
	settings.Lflag &^= syscall.ECHOCTL
	settings.Lflag &^= syscall.ECHOPRT
	settings.Lflag &^= syscall.ECHOKE
	settings.Lflag &^= syscall.ISIG
	settings.Lflag &^= syscall.IEXTEN

	settings.Iflag &^= syscall.IXON
	settings.Iflag &^= syscall.IXOFF
	settings.Iflag &^= syscall.IXANY
	settings.Iflag &^= syscall.INPCK
	settings.Iflag &^= syscall.IGNPAR
	settings.Iflag &^= syscall.PARMRK
	settings.Iflag &^= syscall.ISTRIP
	settings.Iflag &^= syscall.IGNBRK
	settings.Iflag &^= syscall.BRKINT
	settings.Iflag &^= syscall.INLCR
	settings.Iflag &^= syscall.IGNCR
	settings.Iflag &^= syscall.ICRNL
	settings.Iflag &^= syscall.IUCLC

	settings.Oflag &^= syscall.OPOST

	// Block reads until at least one char is available (no timeout)
	settings.Cc[syscall.VMIN] = 1
	settings.Cc[syscall.VTIME] = 0
}

func setTermSettingsBaudrate(baudrate uint32, settings *syscall.Termios) error {
	// revert old baudrate
	for _, rate := range acceptedBauds {
		settings.Cflag &^= rate
	}
	// set new baudrate
	settings.Cflag |= baudrate
	settings.Ispeed = baudrate
	settings.Ospeed = baudrate
	return nil
}

func setTermSettingsParity(parity uart.Parity, settings *syscall.Termios) error {
	switch parity {
	case uart.NoParity:
		settings.Cflag &^= syscall.PARENB
		settings.Cflag &^= syscall.PARODD
		settings.Iflag &^= syscall.INPCK
	case uart.Odd:
		settings.Cflag |= syscall.PARENB
		settings.Cflag |= syscall.PARODD
		settings.Iflag |= syscall.INPCK
	case uart.Even:
		settings.Cflag |= syscall.PARENB
		settings.Cflag &^= syscall.PARODD
		settings.Iflag |= syscall.INPCK
	case uart.Mark:
		return errors.New("sysfs-uart: mark parity is not supported")
	case uart.Space:
		return errors.New("sysfs-uart: space parity is not supported")
	default:
		return fmt.Errorf("sysfs-uart: invalid parity %d", parity)
	}
	return nil
}

func setTermSettingsDataBits(bits int, settings *syscall.Termios) error {
	databits, ok := databitsMap[bits]
	if !ok {
		return fmt.Errorf("sysfs-uart: invalid data bits %d", bits)
	}
	// Remove previous databits setting
	settings.Cflag &^= syscall.CSIZE
	// Set requested databits
	settings.Cflag |= databits
	return nil
}

func setTermSettingsStopBits(bits uart.Stop, settings *syscall.Termios) error {
	switch bits {
	case uart.One:
		settings.Cflag &^= syscall.CSTOPB
	case uart.OneHalf:
		return fmt.Errorf("sysfs-uart: invalid data bits %d", bits)
	case uart.Two:
		settings.Cflag |= syscall.CSTOPB
	default:
		return fmt.Errorf("sysfs-uart: invalid data bits %d", bits)
	}
	return nil
}

func ioctl(f uintptr, op uint, arg uintptr) error {
	if _, _, errno := syscall.Syscall(syscall.SYS_IOCTL, f, uintptr(op), arg); errno != 0 {
		return syscall.Errno(errno)
	}
	return nil
}

func newPortCloserDevFs(name string) (uart.PortCloser, error) {
	// TODO: set port number
	// Use the devfs path for now.

	f, err := os.OpenFile(name, os.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, os.ModeExclusive)
	syscall.SetNonblock(int(f.Fd()), false)
	if err != nil {
		return nil, err
	}
	p := &Port{serialConn{name: name, f: f, portNumber: -1}}
	return p, nil
}

// Port is an open serial port.
type Port struct {
	conn serialConn
}

// Close implements uart.PortCloser.
func (p *Port) Close() error {
	err := p.conn.f.Close()
	p.conn.f = nil
	return err
}

// String implements uart.Port.
func (p *Port) String() string {
	return p.conn.String()
}

// Connect implements uart.Port.
func (p *Port) Connect(f physic.Frequency, stopBit uart.Stop, parity uart.Parity, flow uart.Flow, bits int) (uart.Conn, error) {
	if f > physic.GigaHertz {
		return nil, fmt.Errorf("sysfs-uart: invalid speed %s; maximum supported clock is 1GHz", f)
	}
	if f < 50*physic.Hertz {
		return nil, fmt.Errorf("sysfs-uart: invalid speed %s; minimum supported clock is 50Hz; did you forget to multiply by physic.Hertz?", f)
	}
	if bits < 5 || bits > 8 {
		return nil, fmt.Errorf("sysfs-uart: invalid bits %d; must be between 5 and 8", bits)
	}

	settings, err := p.getTermSettings()
	if err != nil {
		return nil, err
	}
	baud := uint32(f / physic.Hertz)

	baudRateTermios, ok := acceptedBauds[baud]
	if !ok {
		return nil, fmt.Errorf("sysfs-uart: invalid baud rate", f)
	}

	p.conn.mu.Lock()
	defer p.conn.mu.Unlock()

	if p.conn.f == nil {
		return nil, errors.New("sysfs-uart: already closed")
	}
	if p.conn.connected {
		return nil, errors.New("sysfs-uart: already connected")
	}

	setRawMode(settings)

	err = setTermSettingsBaudrate(baudRateTermios, settings)
	if err != nil {
		return nil, err
	}
	p.conn.freqConn = f

	err = setTermSettingsDataBits(bits, settings)
	if err != nil {
		return nil, err
	}
	p.conn.bitsPerWord = uint8(bits)

	err = setTermSettingsParity(parity, settings)
	if err != nil {
		return nil, err
	}

	err = setTermSettingsStopBits(stopBit, settings)
	if err != nil {
		return nil, err
	}

	if flow != uart.NoFlow {
		return nil, errors.New("sysfs-uart: only no flow control is currently supported")
	}

	if flow != uart.RTSCTS {
		p.conn.muPins.Lock()
		p.conn.rts = gpio.INVALID
		p.conn.cts = gpio.INVALID
		p.conn.muPins.Unlock()
	}

	err = p.setTermSettings(settings)
	if err != nil {
		return nil, err
	}

	return &p.conn, nil
}

// LimitSpeed implements uart.PortCloser.
func (p *Port) LimitSpeed(f physic.Frequency) error {
	if f > physic.GigaHertz {
		return fmt.Errorf("sysfs-uart: invalid speed %s; maximum supported clock is 1GHz", f)
	}
	if f < 50*physic.Hertz {
		return fmt.Errorf("sysfs-uart: invalid speed %s; minimum supported clock is 50Hz; did you forget to multiply by physic.KiloHertz?", f)
	}
	p.conn.mu.Lock()
	defer p.conn.mu.Unlock()
	p.conn.freqPort = f
	return nil
}

// RX implements uart.Pins.
func (p *Port) RX() gpio.PinIn {
	return p.conn.RX()
}

// TX implements uart.Pins.
func (p *Port) TX() gpio.PinOut {
	return p.conn.TX()
}

// RTS implements uart.Pins.
func (p *Port) RTS() gpio.PinOut {
	return p.conn.RTS()
}

// CTS implements uart.Pins.
func (p *Port) CTS() gpio.PinIn {
	return p.conn.CTS()
}

func (p *Port) getTermSettings() (*syscall.Termios, error) {
	var value syscall.Termios
	err := ioctl(p.conn.f.Fd(), syscall.TCGETS, uintptr(unsafe.Pointer(&value)))
	return &value, err
}

func (p *Port) setTermSettings(settings *syscall.Termios) error {
	err := ioctl(p.conn.f.Fd(), syscall.TCSETS, uintptr(unsafe.Pointer(settings)))
	runtime.KeepAlive(settings)
	return err
}

type serialConn struct {
	// Immutable
	name       string
	f          *os.File
	portNumber int

	mu          sync.Mutex
	freqPort    physic.Frequency // Frequency specified at LimitSpeed()
	freqConn    physic.Frequency // Frequency specified at Connect()
	bitsPerWord uint8
	connected   bool

	// Use a separate lock for the pins, so that they can be queried while a
	// transaction is happening.
	muPins sync.Mutex
	rx     gpio.PinIn
	tx     gpio.PinOut
	rts    gpio.PinOut
	cts    gpio.PinIn
}

// String implements conn.Conn.
func (s *serialConn) String() string {
	return s.name
}

// Duplex implements conn.Conn.
func (s *serialConn) Duplex() conn.Duplex {
	return conn.Full
}

// Read implements io.Reader.
func (s *serialConn) Read(b []byte) (int, error) {
	return s.f.Read(b)
}

// Write implements io.Writer.
func (s *serialConn) Write(b []byte) (int, error) {
	return s.f.Write(b)
}

// Tx implements conn.Conn.
func (s *serialConn) Tx(w, r []byte) error {
	if len(w) != 0 {
		if _, err := s.f.Write(w); err != nil {
			return err
		}
	}
	if len(r) != 0 {
		_, err := s.f.Read(r)
		return err
	}
	return nil
}

// RX implements uart.Pins.
func (s *serialConn) RX() gpio.PinIn {
	s.initPins()
	return s.rx
}

// TX implements uart.Pins.
func (s *serialConn) TX() gpio.PinOut {
	s.initPins()
	return s.tx
}

// RTS implements uart.Pins.
func (s *serialConn) RTS() gpio.PinOut {
	s.initPins()
	return s.rts
}

// CTS implements uart.Pins.
func (s *serialConn) CTS() gpio.PinIn {
	s.initPins()
	return s.cts
}

func (s *serialConn) initPins() {
	s.muPins.Lock()
	defer s.muPins.Unlock()
	if s.rx != nil {
		return
	}
	if s.rx = gpioreg.ByName(fmt.Sprintf("UART%d_RX", s.portNumber)); s.rx == nil {
		s.rx = gpio.INVALID
	}
	if s.tx = gpioreg.ByName(fmt.Sprintf("UART%d_TX", s.portNumber)); s.tx == nil {
		s.tx = gpio.INVALID
	}
	// s.rts is set to INVALID if no hardware RTS/CTS flow control is used.
	if s.rts == nil {
		if s.rts = gpioreg.ByName(fmt.Sprintf("UART%d_RTS", s.portNumber)); s.rts == nil {
			s.rts = gpio.INVALID
		}
		if s.cts = gpioreg.ByName(fmt.Sprintf("UART%d_CTS", s.portNumber)); s.cts == nil {
			s.cts = gpio.INVALID
		}
	}
}

//

// driverSerial implements periph.Driver.
type driverSerial struct {
}

func (d *driverSerial) String() string {
	return "serial"
}

func (d *driverSerial) Prerequisites() []string {
	return nil
}

func (d *driverSerial) After() []string {
	return nil
}

func (d *driverSerial) Init() (bool, error) {
	prefixes := []string{"/dev/ttyS", "/dev/ttyUSB", "/dev/ttyA"}
	items := make([]string, 0)
	for _, prefix := range prefixes {
		itemForPrefix, err := filepath.Glob(prefix + "*")
		if err != nil {
			return true, err
		}
		items = append(items, itemForPrefix...)
	}

	if len(items) == 0 {
		return false, errors.New("no uart devices found")
	}
	// Make sure they are registered in order.
	sort.Strings(items)
	for _, item := range items {
		port := -1
		//todo change to A
		r, err := regexp.Compile(`/dev/ttyUSB([\d]+)`)
		if err != nil {
			return true, err
		}
		matches := r.FindStringSubmatch(item)
		if len(matches) == 2 {
			port, err = strconv.Atoi(matches[1])
			if err != nil {
				return true, err
			}
		}
		if err := uartreg.Register(item, []string{}, port, openerUart(item).Open); err != nil {
			return true, err
		}
	}
	return true, nil
}

type openerUart string

func (o openerUart) Open() (uart.PortCloser, error) {
	b, err := newPortCloserDevFs(string(o))
	if err != nil {
		return nil, err
	}
	return b, nil
}

func init() {
	driverreg.MustRegister(&drv)
}

var drv driverSerial

var _ uart.PortCloser = &Port{}
var _ uart.Pins = &Port{}
var _ conn.Conn = &serialConn{}
var _ uart.Pins = &serialConn{}
