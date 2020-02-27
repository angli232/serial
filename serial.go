// Package serial provides a portable interface to access serial ports.
package serial

import (
	"errors"
)

type Parity int
type StopBits int
type FlowControl int

// Parity checking.
const (
	ParityNone Parity = iota
	ParityOdd
	ParityEven
	ParityMark
	ParitySpace
)

// Number of stop bits.
const (
	StopBitsOne StopBits = iota
	StopBitsOnePointFive
	StopBitsTwo
)

func (p Parity) String() string {
	switch p {
	case ParityNone:
		return "none"
	case ParityOdd:
		return "odd"
	case ParityEven:
		return "even"
	case ParityMark:
		return "mark"
	case ParitySpace:
		return "space"
	}
	return ""
}

func (s StopBits) String() string {
	switch s {
	case StopBitsOne:
		return "1"
	case StopBitsOnePointFive:
		return "1.5"
	case StopBitsTwo:
		return "2"
	}
	return ""
}

// Flow control used during transfering.
//
// If no flow control is used, buffers on both sides of the connection are assumed to be large enough.
//
// Software (XON/XOFF) flow control uses XON and XOFF character to perform flow control.
//
// Hardware (RTS/CTS) flow control uses RTS output and CTS input signal to perform flow control.
//
// Hardware (DTR/DSR) flow control uses DTR output and DSR input signal to perform flow control.
const (
	FlowControlNone    FlowControl = 0         // No flow control. (Default)
	FlowControlXonXoff FlowControl = 1 << iota // Software (XON/XOFF) flow control.
	FlowControlRtsCts                          // Hardware (RTS/CTS) flow control.
	FlowControlDtrDsr                          // Hardware (DTR/DSR) flow control.
)

// Default XON/XOFF character.
const (
	xON  = 17
	xOFF = 19
)

type Config struct {
	BaudRate    int         // Baud rate. (Default: 9600)
	DataBits    int         // Number of data bits. Range: 5-8. (Default: 8)
	Parity      Parity      // Parity checking.
	StopBits    StopBits    // Number of stop bits.
	FlowControl FlowControl // Flow control mechanism.
}

var defaultConfig Config = Config{
	BaudRate:    9600,
	DataBits:    8,
	Parity:      ParityNone,
	StopBits:    StopBitsOne,
	FlowControl: FlowControlNone,
}

var defaultTimeoutFirstByte = 100
var defaultTimeoutInterByte = 0

var (
	ErrTimeout      = errors.New("I/O operation timeout")
	ErrInvalid      = errors.New("invalid")
	ErrInvalidName  = errors.New("invalid name")
	ErrInvalidParam = errors.New("invalid parameter")
	ErrNotSupported = errors.New("not supported by OS or implementation")
	ErrUnexpected   = errors.New("unexpected response")
)
