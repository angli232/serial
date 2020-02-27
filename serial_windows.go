package serial

// https://msdn.microsoft.com/en-us/library/ff802693.aspx

import (
	"math"
	"regexp"
	"runtime"
	"sync"
	"time"
	"unsafe"

	"golang.org/x/sys/windows"
	"golang.org/x/sys/windows/registry"
)

var (
	modkernel32 = windows.NewLazyDLL("kernel32.dll")

	procSetCommState        = modkernel32.NewProc("SetCommState")
	procGetCommState        = modkernel32.NewProc("GetCommState")
	procSetupComm           = modkernel32.NewProc("SetupComm")
	procSetCommTimeouts     = modkernel32.NewProc("SetCommTimeouts")
	procGetOverlappedResult = modkernel32.NewProc("GetOverlappedResult")
	procPurgeComm           = modkernel32.NewProc("PurgeComm")
)

type Port struct {
	sync.Mutex
	name              string
	h                 windows.Handle
	rl                sync.Mutex
	wl                sync.Mutex
	baudrate          int
	timeout_firstbyte int
	timeout_interbyte int
}

// Flags for PurgeComm.
const (
	purgeTxAbort = 1 << 0 // Terminates all outstanding overlapped write operations and returns immediately, even if the write operations have not been completed.
	purgeRxAbort = 1 << 1 // Terminates all outstanding overlapped read operations and returns immediately, even if the read operations have not been completed.
	purgeTxClear = 1 << 2 // Clears the output buffer (if the device driver has one).
	purgeRxClear = 1 << 3 // Clears the input buffer (if the device driver has one).
)

func isPortNameValid(name string) bool {
	matched, _ := regexp.MatchString("^[A-Za-z]+[0-9]+$", name)
	return matched
}

// formatPortName formats the port name to prepare it for CreatFile call.
func formatPortName(name string) string {
	matched, _ := regexp.MatchString("^COM[1-9]$", name)
	if matched {
		return name
	}
	return `\\.\` + name
}

func ListPortNames() ([]string, error) {
	k, err := registry.OpenKey(registry.LOCAL_MACHINE, `HARDWARE\DEVICEMAP\SERIALCOMM`, registry.READ)
	if err != nil {
		return nil, err
	}
	defer k.Close()

	value_names, err := k.ReadValueNames(-1)
	if err != nil {
		return nil, err
	}

	port_names := make([]string, 0)
	for _, value_name := range value_names {
		d, _, err := k.GetStringValue(value_name)
		if err != nil {
			continue
		}
		port_names = append(port_names, d)
	}
	return port_names, err
}

func Open(name string, c *Config) (p *Port, err error) {
	if !isPortNameValid(name) {
		return nil, ErrInvalidName
	}

	namePtr, _ := windows.UTF16PtrFromString(formatPortName(name))

	// Open the serial port.
	h, err := windows.CreateFile(namePtr,
		windows.GENERIC_READ|windows.GENERIC_WRITE,
		0,
		nil,
		windows.OPEN_EXISTING,
		windows.FILE_ATTRIBUTE_NORMAL|windows.FILE_FLAG_OVERLAPPED,
		0)
	if h == windows.InvalidHandle {
		return nil, ErrInvalidName
	}
	if err != nil {
		return nil, err
	}
	defer func() {
		if err != nil {
			windows.CloseHandle(h)
		}
	}()

	// TODO Check whether it is a serial port.

	p = &Port{
		name: name,
		h:    h,
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
	dcb, err := buildDCB(c)
	if err != nil {
		return nil, err
	}
	err = p.setCommState(dcb)
	if err != nil {
		return nil, err
	}

	// Set internal buffer size.
	err = p.setupComm(4096, 4096)
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
	if p == nil || p.h == windows.InvalidHandle {
		return nil
	}

	windows.CloseHandle(p.h)
	p.h = windows.InvalidHandle

	// We no longer need the finalizer.
	runtime.SetFinalizer(p, nil)
	return nil
}

func (p *Port) overlappedRead(buf []byte) (int, error) {
	// Create overlapped structure
	var overlapped windows.Overlapped
	var err error
	overlapped.HEvent, err = windows.CreateEvent(nil, 1, 0, nil)
	if err != nil {
		return 0, err
	}
	defer windows.CloseHandle(overlapped.HEvent)

	// Issue read operation.
	var n uint32
	err = windows.ReadFile(p.h, buf, &n, &overlapped)
	if err == nil {
		// ReadFile completed immediately
		return int(n), nil
	}
	if err != windows.ERROR_IO_PENDING {
		// ReadFile failed
		return int(n), err
	}
	// Wait and get the results
	r, _, err := procGetOverlappedResult.Call(uintptr(p.h),
		uintptr(unsafe.Pointer(&overlapped)),
		uintptr(unsafe.Pointer(&n)), 1)
	if r == 0 {
		return int(n), err
	}
	return int(n), nil
}

func (p *Port) Read(buf []byte) (int, error) {
	if p == nil || p.h == windows.InvalidHandle {
		return 0, ErrInvalid
	}
	p.rl.Lock()
	defer p.rl.Unlock()

	if len(buf) == 0 {
		return 0, nil
	}
	if len(buf) == 1 {
		// Read with total timeout only
		p.setCommTimeouts(p.timeout_firstbyte, 0)
		n, err := p.overlappedRead(buf)
		if err != nil && n == 0 {
			return 0, ErrTimeout
		}
		return n, err
	}

	// When no interbyte timeout is specified, set it based on baudrate
	timeout_interbyte := p.timeout_interbyte
	if timeout_interbyte == 0 {
		timeout_interbyte = int(math.Ceil(1000 / (float64(p.baudrate) / 10) * 1.5))
	}

	// Read first byte with total timeout
	err := p.setCommTimeouts(p.timeout_firstbyte, 0)
	if err != nil {
		return 0, err
	}
	n, err := p.overlappedRead(buf[0:1])
	if n == 0 {
		if err != nil {
			return 0, err
		}
		return 0, ErrTimeout
	}

	// Data did arrive.
	// Perform non-blocking read from OS,
	// and manage interbyte timeout ourselves,
	// since the interval timeout of OS is active after the first byte arrives.
	err = p.setCommTimeouts(0, -1)
	if err != nil {
		return n, err
	}

	for n < len(buf) {
		time.Sleep(time.Duration(timeout_interbyte) * time.Millisecond)
		delta, err := p.overlappedRead(buf[n:len(buf)])
		n += delta
		if delta == 0 || err != nil {
			return n, err
		}
	}

	return n, err
}

func (p *Port) Write(buf []byte) (int, error) {
	if p == nil || p.h == windows.InvalidHandle {
		return 0, ErrInvalid
	}
	p.wl.Lock()
	defer p.wl.Unlock()

	// Create overlapped structure
	var overlapped windows.Overlapped
	var err error
	overlapped.HEvent, err = windows.CreateEvent(nil, 1, 0, nil)
	if err != nil {
		return 0, err
	}
	defer windows.CloseHandle(overlapped.HEvent)

	// Issue write operation.
	var n uint32
	err = windows.WriteFile(p.h, buf, &n, &overlapped)
	if err == nil {
		// WriteFile completed immediately
		return int(n), err
	}
	if err != windows.ERROR_IO_PENDING {
		// WriteFile failed
		return int(n), err
	}
	// Write is pending
	r, _, err := procGetOverlappedResult.Call(uintptr(p.h),
		uintptr(unsafe.Pointer(&overlapped)),
		uintptr(unsafe.Pointer(&n)), 1)
	if r == 0 {
		return int(n), err
	}
	return int(n), nil
}

// Flush discards data written to the port but not transmitted,
// or data received but not read.
func (p *Port) Flush() error {
	if p == nil || p.h == windows.InvalidHandle {
		return ErrInvalid
	}

	r, _, err := procPurgeComm.Call(uintptr(p.h),
		purgeTxAbort|purgeRxAbort|purgeTxClear|purgeRxClear)
	if r == 0 {
		return err
	}
	return nil
}

func (p *Port) SetBaudRate(baudrate int) error {
	dcb, err := p.getCommState()
	if err != nil {
		return err
	}
	dcb.BaudRate = uint32(baudrate)
	return p.setCommState(dcb)
}

func (p *Port) SetDataBits(databits int) error {
	dcb, err := p.getCommState()
	if err != nil {
		return err
	}
	dcb.ByteSize = uint8(databits)
	return p.setCommState(dcb)
}

func (p *Port) SetParity(parity Parity) error {
	dcb, err := p.getCommState()
	if err != nil {
		return err
	}
	err = dcb.SetParity(parity)
	if err != nil {
		return err
	}
	return p.setCommState(dcb)
}

func (p *Port) SetStopBits(stopbits StopBits) error {
	dcb, err := p.getCommState()
	if err != nil {
		return err
	}
	err = dcb.SetStopBits(stopbits)
	if err != nil {
		return err
	}
	return p.setCommState(dcb)
}

func (p *Port) SetFlowControl(flowcontrol FlowControl) error {
	dcb, err := p.getCommState()
	if err != nil {
		return err
	}
	err = dcb.SetFlowControl(flowcontrol)
	if err != nil {
		return err
	}
	return p.setCommState(dcb)
}

func (p *Port) BaudRate() (int, error) {
	dcb, err := p.getCommState()
	if err != nil {
		return 0, err
	}
	return int(dcb.BaudRate), nil
}

func (p *Port) DataBits() (int, error) {
	dcb, err := p.getCommState()
	if err != nil {
		return 0, err
	}
	return int(dcb.ByteSize), nil
}

func (p *Port) Parity() (Parity, error) {
	dcb, err := p.getCommState()
	if err != nil {
		return 0, err
	}
	switch dcb.Parity {
	case 0:
		return ParityNone, nil
	case 1:
		return ParityOdd, nil
	case 2:
		return ParityEven, nil
	case 3:
		return ParityMark, nil
	case 4:
		return ParitySpace, nil
	}
	return ParityNone, nil
}

func (p *Port) StopBits() (StopBits, error) {
	dcb, err := p.getCommState()
	if err != nil {
		return 0, err
	}
	switch dcb.StopBits {
	case 0:
		return StopBitsOne, nil
	case 1:
		return StopBitsOnePointFive, nil
	case 2:
		return StopBitsTwo, nil
	}
	return StopBitsOne, nil
}

func (p *Port) FlowControl() FlowControl {
	return 0
}

// setupComm sets up the recommended size of the device's internal input/output buffer, in bytes.
func (p *Port) setupComm(in, out uint32) error {
	r, _, err := procSetupComm.Call(uintptr(p.h), uintptr(in), uintptr(out))
	if r == 0 {
		return err
	}
	return nil
}

// Bits for Flags in winDCB.
const (
	fBinary              = 1 << 0     // Binary mode transfer, must be true.
	fParity              = 1 << 1     // Enable parity checking.
	fOutxCtsFlow         = 1 << 2     // Monitor CTS (clear-to-send).
	fOutxDsrFlow         = 1 << 3     // Monitor DSR (data-set-ready).
	fDtrControlMask      = 0x03 << 4  // Bit mask for DTR Control.
	fDtrControlEnable    = 0x01 << 4  // Enable DTR line.
	fDtrControlHandshake = 0x02 << 4  // Enable DTR handshaking.
	fDsrSensitivity      = 1 << 6     // Sensitive to DSR signal.
	fTXContinueOnXoff    = 1 << 7     //
	fOutX                = 1 << 8     // Use XON/XOFF flow control during transmission.
	fInX                 = 1 << 9     // USE XON/XOFF flow control during reception.
	fErrorChar           = 1 << 10    // Error character being replaced at parity errors.
	fNull                = 1 << 11    // Discard null bytes.
	fRtsControlMask      = 0x03 << 12 // Bit mask for RTS Control.
	fRtsControlEnable    = 0x01 << 12 // Enable RTS line.
	fRtsControlHandshake = 0x02 << 12 // Enable RTS handshaking.
	fRtsControlToggle    = 0x03 << 12 //
	fAbortOnError        = 1 << 14    // Abort read and write on error.
)

type winDCB struct {
	DCBlength uint32
	BaudRate  uint32
	Flags     uint32
	reserved  uint16
	XonLim    uint16
	XoffLim   uint16
	ByteSize  uint8
	Parity    uint8
	StopBits  uint8
	XonChar   byte
	XoffChar  byte
	ErrorChar byte
	EofChar   byte
	EvtChar   byte
	reserved1 uint16
}

type commTimeouts struct {
	ReadIntervalTimeout         uint32
	ReadTotalTimeoutMultiplier  uint32
	ReadTotalTimeoutConstant    uint32
	WriteTotalTimeoutMultiplier uint32
	WriteTotalTimeoutConstant   uint32
}

// total=0, interval=-1 for non-blocking read
// total=0, interval>0 for interval timeout only
// total>0, interval=0 for total timeout only
func (p *Port) setCommTimeouts(total, interval int) error {
	var timeouts commTimeouts
	if interval == -1 {
		// Non-blocking read
		timeouts.ReadIntervalTimeout = 2<<31 - 1
	} else {
		timeouts.ReadIntervalTimeout = uint32(interval)
		timeouts.ReadTotalTimeoutConstant = uint32(total)
	}
	r, _, err := procSetCommTimeouts.Call(uintptr(p.h), uintptr(unsafe.Pointer(&timeouts)))
	if r == 0 {
		return err
	}
	return nil
}

// setCommState sets the DCB structure which defines the control settings for a serial device.
func (p *Port) setCommState(dcb *winDCB) error {
	r, _, err := procSetCommState.Call(uintptr(p.h), uintptr(unsafe.Pointer(dcb)))
	if r == 0 {
		return err
	}
	return nil
}

// getCommState gets the DCB structure which defines the control settings for a serial device.
func (p *Port) getCommState() (*winDCB, error) {
	if p == nil || p.h == windows.InvalidHandle {
		return nil, ErrInvalid
	}

	var dcb winDCB
	r, _, err := procGetCommState.Call(uintptr(p.h), uintptr(unsafe.Pointer(&dcb)))
	if r == 0 {
		return nil, err
	}
	return &dcb, nil
}

func buildDCB(c *Config) (*winDCB, error) {
	dcb := &winDCB{}
	dcb.DCBlength = uint32(unsafe.Sizeof(dcb))
	dcb.Flags |= fBinary

	dcb.BaudRate = uint32(c.BaudRate)
	dcb.ByteSize = uint8(c.DataBits)
	err := dcb.SetParity(c.Parity)
	if err != nil {
		return nil, err
	}
	err = dcb.SetStopBits(c.StopBits)
	if err != nil {
		return nil, err
	}
	err = dcb.SetFlowControl(c.FlowControl)
	if err != nil {
		return nil, err
	}
	return dcb, nil
}

func (dcb *winDCB) SetParity(parity Parity) error {
	switch parity {
	case ParityNone:
		dcb.Parity = 0
		dcb.Flags &^= fParity
	case ParityOdd:
		dcb.Parity = 1
		dcb.Flags |= fParity
	case ParityEven:
		dcb.Parity = 2
		dcb.Flags |= fParity
	case ParityMark:
		dcb.Parity = 3
		dcb.Flags |= fParity
	case ParitySpace:
		dcb.Parity = 4
		dcb.Flags |= fParity
	default:
		return ErrInvalidParam
	}
	return nil
}

func (dcb *winDCB) SetStopBits(stopbits StopBits) error {
	switch stopbits {
	case StopBitsOne:
		dcb.StopBits = 0
	case StopBitsOnePointFive:
		dcb.StopBits = 1
	case StopBitsTwo:
		dcb.StopBits = 2
	default:
		return ErrInvalidParam
	}
	return nil
}

func (dcb *winDCB) SetFlowControl(flowcontrol FlowControl) error {
	if flowcontrol&FlowControlRtsCts != 0 {
		dcb.Flags |= fRtsControlHandshake | fOutxCtsFlow
	} else {
		dcb.Flags &^= fRtsControlHandshake | fOutxCtsFlow
	}

	if flowcontrol&FlowControlDtrDsr != 0 {
		dcb.Flags |= fDtrControlHandshake | fOutxDsrFlow
	} else {
		dcb.Flags &^= fDtrControlHandshake | fOutxDsrFlow
	}

	dcb.XonChar = xON
	dcb.XoffChar = xOFF
	if flowcontrol&FlowControlXonXoff != 0 {
		dcb.Flags |= fInX | fOutX
	} else {
		dcb.Flags &^= fInX | fOutX
	}
	return nil
}

func (p *Port) SetTimeout(timeout_ms_firstbyte, timeout_ms_interbyte int) error {
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
