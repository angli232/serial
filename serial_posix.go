// +build linux darwin dragonfly freebsd netbsd openbsd

package serial

import (
	"io/ioutil"
	"math"
	"os"
	"regexp"
	"runtime"
	"strings"
	"sync"
	"syscall"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

type Port struct {
	sync.Mutex
	f                 *os.File
	rl                sync.Mutex
	wl                sync.Mutex
	baudrate          int
	timeout_firstbyte int
	timeout_interbyte int
}

func ListPortNames() ([]string, error) {
	serial_list := make([]string, 0)

	ls, err := ioutil.ReadDir("/dev")
	if err != nil {
		return serial_list, err
	}

	for _, info := range ls {
		name := info.Name()
		var is_serial bool
		switch runtime.GOOS {
		case "linux":
			// http://tldp.org/HOWTO/Serial-HOWTO-10.html
			// Some non-standard names are currently not supported.
			is_serial = strings.HasPrefix(name, "ttyS") ||
				strings.HasPrefix(name, "ttyUSB") ||
				strings.HasPrefix(name, "ttyACM")
		case "darwin":
			is_serial = strings.HasPrefix(name, "tty.")
		case "freebsd":
			// https://www.freebsd.org/doc/handbook/serial.html
			// Call-in ports are named /dev/ttyuN where N is the port number,
			// starting from zero. If a terminal is connected to the first
			// serial port (COM1), use /dev/ttyu0 to refer to the terminal.
			is_serial, _ = regexp.MatchString("ttyu[0-9]+", name)
		case "openbsd":
			// http://www.openbsd.org/faq/faq7.html#SerCon
			// OpenBSD numbers the serial ports starting at tty00, DOS/Windows
			// labels them starting at COM1. So, keep in mind tty02 is COM3,
			// not COM2.
			is_serial, _ = regexp.MatchString("tty[0-9]+", name)
		case "netbsd":
			// https://www.netbsd.org/docs/guide/en/chap-net-practice.html
			// For NetBSD, “com” is the name of the serial port driver (the one
			// that is displayed by dmesg) and “tty” is the name of the port.
			// Since numbering starts at 0, com0 is the driver for the first
			// serial port, named tty00. In the DOS world, instead, COM1 refers
			// to the first serial port (usually located at 0x3f8), COM2 to the
			// second, and so on. Therefore COM1 (DOS) corresponds to
			// /dev/tty00 (NetBSD).
			is_serial, _ = regexp.MatchString("tty[0-9]+", name)
		case "dragonfly":
			// https://www.dragonflybsd.org/docs/docs/newhandbook/serial_communications/
			// Call-in ports are named /dev/ttydN* where ***N*** is the port
			// number, starting from zero. Generally, you use the call-in port
			// for terminals. Call-in ports require that the serial line assert
			// the data carrier detect (DCD) signal to work correctly.
			is_serial, _ = regexp.MatchString("ttyd[0-9]+", name)
		}
		if is_serial {
			serial_list = append(serial_list, "/dev/"+name)
		}
	}
	return serial_list, nil
}

func Open(name string, c *Config) (p *Port, err error) {
	// Open the serial port.
	f, err := os.OpenFile(name, syscall.O_RDWR|syscall.O_NOCTTY|syscall.O_NONBLOCK, 0666)
	if err != nil {
		return nil, err
	}
	defer func() {
		if err != nil {
			f.Close()
		}
	}()

	p = &Port{f: f}

	// Remove O_NONBLOCK flag after the port is open.
	err = p.setFlag(0)
	if err != nil {
		return nil, err
	}

	// Set the configurations of the serial port.
	if c == nil {
		c = &defaultConfig
	}
	if c.BaudRate == 0 {
		c.BaudRate = defaultConfig.BaudRate
	}
	if c.DataBits == 0 {
		c.DataBits = defaultConfig.DataBits
	}
	t, err := buildTermios(c)
	if err != nil {
		return
	}
	err = p.setTermios(t)
	if err != nil {
		return nil, err
	}

	// Save the baudrate
	p.baudrate = c.BaudRate

	// Set default timeout
	p.SetTimeout(defaultTimeoutFirstByte, defaultTimeoutInterByte)

	// Clear buffers.
	p.Flush()

	// Set a finalizer.
	runtime.SetFinalizer(p, (*Port).Close)
	return p, nil
}

func (p *Port) Close() error {
	if p == nil || p.f == nil {
		return nil
	}

	err := p.f.Close()
	if err != nil {
		return err
	}

	p.f = nil
	runtime.SetFinalizer(p, nil)
	return nil
}

func (p *Port) Read(buf []byte) (int, error) {
	if p == nil || p.f == nil {
		return 0, ErrInvalid
	}
	p.rl.Lock()
	defer p.rl.Unlock()

	if len(buf) == 0 {
		return 0, nil
	}
	if len(buf) == 1 {
		t, err := p.getTermios()
		if err != nil {
			return 0, err
		}
		t.SetTotalTimeout(p.timeout_firstbyte)
		err = p.setTermios(t)
		if err != nil {
			return 0, err
		}
		// TODO support timeout longer than 25.5 s.
		return p.f.Read(buf)
	}

	// When no interbyte timeout is specified, set it based on baudrate
	timeout_interbyte := p.timeout_interbyte
	if timeout_interbyte == 0 {
		timeout_interbyte = int(math.Ceil(1000 / (float64(p.baudrate) / 10) * 1.5))
	}

	// Read first byte with total timeout
	t, err := p.getTermios()
	if err != nil {
		return 0, err
	}
	t.SetTotalTimeout(p.timeout_firstbyte)
	err = p.setTermios(t)
	if err != nil {
		return 0, err
	}
	// TODO support timeout longer than 25.5 s.
	n, err := p.f.Read(buf)
	if n == 0 || err != nil {
		return n, err
	}

	// Data did arrive.
	// Perform non-blocking read from OS,
	// and manage interbyte timeout ourselves.
	t.SetTotalTimeout(0)
	err = p.setTermios(t)
	if err != nil {
		return 0, err
	}

	for n < len(buf) {
		time.Sleep(time.Duration(timeout_interbyte) * time.Millisecond)
		delta, err := p.f.Read(buf[n:len(buf)])
		n += delta
		if delta == 0 || err != nil {
			return n, err
		}
	}
	return n, nil
}

func (p *Port) Write(buf []byte) (int, error) {
	if p == nil || p.f == nil {
		return 0, nil
	}
	p.wl.Lock()
	defer p.wl.Unlock()

	return p.f.Write(buf)
}

// Flush discards data written to the port but not transmitted,
// or data received but not read.
func (p *Port) Flush() error {
	if p == nil || p.f == nil {
		return nil
	}
	return p.flush()
}

func (p *Port) SetBaudRate(baudrate int) error {
	if p == nil || p.f == nil {
		return nil
	}

	p.rl.Lock()
	p.wl.Lock()
	defer func() {
		p.rl.Unlock()
		p.wl.Unlock()
	}()

	t, err := p.getTermios()
	if err != nil {
		return err
	}
	err = t.SetBaudrate(baudrate)
	if err != nil {
		return err
	}
	p.baudrate = baudrate
	return p.setTermios(t)
}

func (p *Port) SetDataBits(databits int) error {
	if p == nil || p.f == nil {
		return nil
	}

	p.rl.Lock()
	p.wl.Lock()
	defer func() {
		p.rl.Unlock()
		p.wl.Unlock()
	}()

	t, err := p.getTermios()
	if err != nil {
		return err
	}
	err = t.SetDataBits(databits)
	if err != nil {
		return err
	}
	return p.setTermios(t)
}

func (p *Port) SetParity(parity Parity) error {
	if p == nil || p.f == nil {
		return nil
	}

	p.rl.Lock()
	p.wl.Lock()
	defer func() {
		p.rl.Unlock()
		p.wl.Unlock()
	}()

	t, err := p.getTermios()
	if err != nil {
		return err
	}
	err = t.SetParity(parity)
	if err != nil {
		return err
	}
	return p.setTermios(t)
}

func (p *Port) SetStopBits(stopbits StopBits) error {
	if p == nil || p.f == nil {
		return nil
	}

	p.rl.Lock()
	p.wl.Lock()
	defer func() {
		p.rl.Unlock()
		p.wl.Unlock()
	}()

	t, err := p.getTermios()
	if err != nil {
		return err
	}
	err = t.SetStopBits(stopbits)
	if err != nil {
		return err
	}
	return p.setTermios(t)
}

func (p *Port) SetFlowControl(flowcontrol FlowControl) error {
	if p == nil || p.f == nil {
		return nil
	}

	p.rl.Lock()
	p.wl.Lock()
	defer func() {
		p.rl.Unlock()
		p.wl.Unlock()
	}()

	t, err := p.getTermios()
	if err != nil {
		return err
	}
	err = t.SetFlowControl(flowcontrol)
	if err != nil {
		return err
	}

	return p.setTermios(t)
}

func (p *Port) SetTimeout(timeout_ms_firstbyte, timeout_ms_interbyte int) error {
	if p == nil || p.f == nil {
		return nil
	}

	p.rl.Lock()
	p.wl.Lock()
	defer func() {
		p.rl.Unlock()
		p.wl.Unlock()
	}()

	if timeout_ms_firstbyte < -1 || timeout_ms_firstbyte > 25500 {
		return ErrInvalidParam
	}
	if timeout_ms_interbyte < 0 {
		return ErrInvalidParam
	}
	p.timeout_firstbyte = timeout_ms_firstbyte
	p.timeout_interbyte = timeout_ms_interbyte
	return nil
}

func (p *Port) BaudRate() (int, error) {
	t, err := p.getTermios()
	if err != nil {
		return 0, nil
	}
	return t.GetBaudrate()
}

func (p *Port) DataBits() (int, error) {
	t, err := p.getTermios()
	if err != nil {
		return 0, nil
	}
	switch t.Cflag & syscall.CSIZE {
	case syscall.CS5:
		return 5, nil
	case syscall.CS6:
		return 6, nil
	case syscall.CS7:
		return 7, nil
	case syscall.CS8:
		return 8, nil
	}
	return 8, ErrUnexpected
}

func (p *Port) Parity() (Parity, error) {
	const CMSPAR = 0x40000000
	t, err := p.getTermios()
	if err != nil {
		return 0, nil
	}
	if t.Cflag&syscall.PARENB == 0 {
		return ParityNone, nil
	}
	if t.Cflag&CMSPAR == 0 {
		if t.Cflag&syscall.PARODD != 0 {
			return ParityOdd, nil
		}
		return ParityEven, nil

	} else {
		if t.Cflag&syscall.PARODD != 0 {
			return ParityMark, nil
		}
		return ParitySpace, nil
	}
	return ParityNone, ErrUnexpected
}

func (p *Port) StopBits() (StopBits, error) {
	t, err := p.getTermios()
	if err != nil {
		return 0, nil
	}
	if t.Cflag&syscall.CSTOPB != 0 {
		return StopBitsTwo, nil
	}
	return StopBitsOne, nil
}

func (p *Port) FlowControl() FlowControl {
	return 0
}

const (
	termios_POSIX_VDISABLE = 0xff
	termios_CCTS_OFLOW     = 0x00010000
	termios_CRTS_IFLOW     = 0x00020000
	termios_CDTR_IFLOW     = 0x00040000
	termios_CDSR_OFLOW     = 0x00080000
	termios_CMSPAR         = 0x40000000
)

type termios unix.Termios

func (p *Port) getTermios() (*termios, error) {
	var t termios
	_, _, e := unix.Syscall6(unix.SYS_IOCTL,
		p.f.Fd(),
		ioctlReadTermios,
		uintptr(unsafe.Pointer(&t)),
		0, 0, 0)
	if e != 0 {
		return nil, e
	}
	return &t, nil
}

func (p *Port) setTermios(t *termios) error {
	_, _, e := unix.Syscall6(unix.SYS_IOCTL,
		p.f.Fd(),
		ioctlWriteTermios,
		uintptr(unsafe.Pointer(t)),
		0, 0, 0)
	if e != 0 {
		return e
	}
	return nil
}

// setFlag performs a fcntl syscall for F_SETFL to set the file status flags.
// File access mode (O_RDONLY, O_WRONLY, O_RDWR) and file creation flags (i.e.,
// O_CREAT, O_EXCL, O_NOCTTY, O_TRUNC) in arg are ignored.
func (p *Port) setFlag(flag int) error {
	_, _, e := unix.Syscall6(unix.SYS_FCNTL,
		p.f.Fd(),
		uintptr(unix.F_SETFL),
		uintptr(flag),
		0, 0, 0)
	if e != 0 {
		return e
	}
	return nil
}

func buildTermios(c *Config) (*termios, error) {
	t := &termios{}
	for i := 0; i < len(t.Cc); i++ {
		t.Cc[i] = termios_POSIX_VDISABLE
	}
	t.Cflag |= unix.CREAD
	err := t.SetBaudrate(c.BaudRate)
	if err != nil {
		return nil, err
	}
	err = t.SetDataBits(c.DataBits)
	if err != nil {
		return nil, err
	}
	err = t.SetParity(c.Parity)
	if err != nil {
		return nil, err
	}
	err = t.SetStopBits(c.StopBits)
	if err != nil {
		return nil, err
	}
	err = t.SetFlowControl(c.FlowControl)
	if err != nil {
		return nil, err
	}
	return t, nil
}

func (t *termios) SetDataBits(databits int) error {
	switch databits {
	case 5:
		t.Cflag = t.Cflag&^unix.CSIZE | unix.CS5
	case 6:
		t.Cflag = t.Cflag&^unix.CSIZE | unix.CS6
	case 7:
		t.Cflag = t.Cflag&^unix.CSIZE | unix.CS7
	case 8:
		t.Cflag = t.Cflag&^unix.CSIZE | unix.CS8
	default:
		return ErrInvalidParam
	}
	return nil
}

func (t *termios) SetParity(parity Parity) error {
	switch parity {
	case ParityNone:
		t.Cflag &^= unix.PARENB
	case ParityOdd:
		t.Cflag |= unix.PARENB
		t.Cflag &^= termios_CMSPAR
		t.Cflag |= unix.PARODD
	case ParityEven:
		t.Cflag |= unix.PARENB
		t.Cflag &^= termios_CMSPAR
		t.Cflag &^= unix.PARODD
	case ParityMark:
		t.Cflag |= unix.PARENB
		t.Cflag |= termios_CMSPAR
		t.Cflag |= unix.PARODD
	case ParitySpace:
		t.Cflag |= unix.PARENB
		t.Cflag |= termios_CMSPAR
		t.Cflag &^= unix.PARODD
	default:
		return ErrInvalidParam
	}
	return nil
}

func (t *termios) SetStopBits(stopbits StopBits) error {
	switch stopbits {
	case StopBitsOne:
		t.Cflag &^= unix.CSTOPB
	case StopBitsOnePointFive:
		return ErrNotSupported
	case StopBitsTwo:
		t.Cflag |= unix.CSTOPB
	default:
		return ErrInvalidParam
	}
	return nil
}

func (t *termios) SetFlowControl(flowcontrol FlowControl) error {
	if flowcontrol&FlowControlRtsCts != 0 {
		t.Cflag |= termios_CRTS_IFLOW | termios_CCTS_OFLOW
	} else {
		t.Cflag &^= termios_CRTS_IFLOW | termios_CCTS_OFLOW
	}

	if flowcontrol&FlowControlDtrDsr != 0 {
		t.Cflag |= termios_CDTR_IFLOW | termios_CDSR_OFLOW
	} else {
		t.Cflag &^= termios_CDTR_IFLOW | termios_CDSR_OFLOW
	}

	t.Cc[unix.VSTART] = xON
	t.Cc[unix.VSTOP] = xOFF
	if flowcontrol&FlowControlXonXoff != 0 {
		t.Iflag |= unix.IXON | unix.IXOFF
	} else {
		t.Iflag &^= unix.IXON | unix.IXOFF
	}

	return nil
}

// timeout_ms = -1 for infinity block
// timeout_ms = 0  for non-block
// timeout_ms > 0  for pure timed read (0 to 25.5 seconds in 0.1 s intervals)
func (t *termios) SetTotalTimeout(timeout_ms int) error {
	switch {
	case timeout_ms == -1:
		t.Cc[unix.VMIN] = 1
		t.Cc[unix.VTIME] = 0
	case timeout_ms == 0:
		t.Cc[unix.VMIN] = 0
		t.Cc[unix.VTIME] = 0
	case timeout_ms > 25500:
		return ErrInvalidParam
	case timeout_ms > 0:
		t.Cc[unix.VMIN] = 0
		t.Cc[unix.VTIME] = uint8(timeout_ms / 100)
	default:
		return ErrInvalidParam
	}
	return nil
}
