param(
    [string]$PortName = "",
    [int]$BaudRate = 115200
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

Add-Type @"
using System;
using System.Runtime.InteropServices;

public static class KeyboardState
{
    [DllImport("user32.dll")]
    private static extern short GetAsyncKeyState(int virtualKey);

    public static bool IsDown(int virtualKey)
    {
        return (GetAsyncKeyState(virtualKey) & 0x8000) != 0;
    }
}
"@

$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object

if ([string]::IsNullOrWhiteSpace($PortName)) {
    if ($availablePorts.Count -eq 0) {
        throw "No COM port found. Check the Nucleo USB connection."
    }

    Write-Host "Available COM ports: $($availablePorts -join ', ')"
    $PortName = Read-Host "Enter the Nucleo COM port (example: COM5)"
}

$serial = [System.IO.Ports.SerialPort]::new(
    $PortName,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.WriteTimeout = 200
$serial.ReadTimeout = 1
$serial.Open()

$VK_ESCAPE = 0x1B
$VK_SPACE = 0x20
$VK_1 = 0x31
$VK_2 = 0x32
$VK_LEFT = 0x25
$VK_UP = 0x26
$VK_RIGHT = 0x27
$VK_DOWN = 0x28
$VK_A = 0x41
$VK_D = 0x44
$VK_E = 0x45
$VK_G = 0x47
$VK_I = 0x49
$VK_J = 0x4A
$VK_K = 0x4B
$VK_O = 0x4F
$VK_Q = 0x51
$VK_S = 0x53
$VK_T = 0x54
$VK_U = 0x55
$VK_W = 0x57

$lastCommand = ""
$rxBuffer = ""
$telemetryText = "RPMx10: waiting | PWM: waiting"

Write-Host ""
Write-Host "STM32 manual drive control started"
Write-Host "Arrow keys/WASD: move, Q/E: rotate, Space: stop, Esc: exit"
Write-Host "Servo 1 PB8: U/J, Servo 2 PB9: I/K, Both: T close/G open"
Write-Host "O: disable both servo PWM outputs"
Write-Host "Ultrasonic sensor 1/2: number keys 1/2"
Write-Host "Releasing the key stops all motors."
Write-Host "Releasing a servo key holds its current position."

try {
    while (-not [KeyboardState]::IsDown($VK_ESCAPE)) {
        $command = "X"
        $label = "STOP"

        if ([KeyboardState]::IsDown($VK_SPACE)) {
            $command = "X"
            $label = "STOP"
        }
        elseif ([KeyboardState]::IsDown($VK_O)) {
            $command = "O"
            $label = "SERVO PWM OFF"
        }
        elseif ([KeyboardState]::IsDown($VK_1)) {
            $command = "1"
            $label = "ULTRASONIC 1"
        }
        elseif ([KeyboardState]::IsDown($VK_2)) {
            $command = "2"
            $label = "ULTRASONIC 2"
        }
        elseif ([KeyboardState]::IsDown($VK_Q)) {
            $command = "Q"
            $label = "ROTATE LEFT"
        }
        elseif ([KeyboardState]::IsDown($VK_E)) {
            $command = "E"
            $label = "ROTATE RIGHT"
        }
        elseif ([KeyboardState]::IsDown($VK_T)) {
            $command = "T"
            $label = "BOTH SERVOS CLOSE"
        }
        elseif ([KeyboardState]::IsDown($VK_G)) {
            $command = "G"
            $label = "BOTH SERVOS OPEN"
        }
        elseif ([KeyboardState]::IsDown($VK_U)) {
            $command = "U"
            $label = "SERVO 1 +"
        }
        elseif ([KeyboardState]::IsDown($VK_J)) {
            $command = "J"
            $label = "SERVO 1 -"
        }
        elseif ([KeyboardState]::IsDown($VK_I)) {
            $command = "I"
            $label = "SERVO 2 +"
        }
        elseif ([KeyboardState]::IsDown($VK_K)) {
            $command = "K"
            $label = "SERVO 2 -"
        }
        elseif ([KeyboardState]::IsDown($VK_UP) -or [KeyboardState]::IsDown($VK_W)) {
            $command = "W"
            $label = "FORWARD"
        }
        elseif ([KeyboardState]::IsDown($VK_DOWN) -or [KeyboardState]::IsDown($VK_S)) {
            $command = "S"
            $label = "BACKWARD"
        }
        elseif ([KeyboardState]::IsDown($VK_LEFT) -or [KeyboardState]::IsDown($VK_A)) {
            $command = "A"
            $label = "STRAFE LEFT"
        }
        elseif ([KeyboardState]::IsDown($VK_RIGHT) -or [KeyboardState]::IsDown($VK_D)) {
            $command = "D"
            $label = "STRAFE RIGHT"
        }

        $serial.Write($command)
        $displayChanged = ($command -ne $lastCommand)

        if (($command -eq "X") -and ($lastCommand -ne "X")) {
            $telemetryText = "RPMx10: stopping | PWM FL=0 FR=0 RL=0 RR=0"
        }

        $incoming = $serial.ReadExisting()

        if (-not [string]::IsNullOrEmpty($incoming)) {
            $rxBuffer += $incoming

            while ($rxBuffer.Contains("`n")) {
                $newlineIndex = $rxBuffer.IndexOf("`n")
                $line = $rxBuffer.Substring(0, $newlineIndex).Trim()
                $rxBuffer = $rxBuffer.Substring($newlineIndex + 1)
                $fields = $line.Split(",")

                if (($fields.Count -eq 14) -and
                    ($fields[0] -eq "T") -and
                    ($fields[1] -eq $command)) {
                    if (($command -eq "1") -or ($command -eq "2")) {
                        $telemetryText = "US mm #1=$($fields[12]) #2=$($fields[13])"
                    }
                    else {
                        $telemetryText =
                            "RPMx10 FL=$($fields[2]) FR=$($fields[3]) RL=$($fields[4]) RR=$($fields[5])" +
                            " | PWM FL=$($fields[6]) FR=$($fields[7]) RL=$($fields[8]) RR=$($fields[9])" +
                            " | SERVO us S1=$($fields[10]) S2=$($fields[11])" +
                            " | US mm #1=$($fields[12]) #2=$($fields[13])"
                    }
                    $displayChanged = $true
                }
            }
        }

        if ($displayChanged) {
            Write-Host -NoNewline "`rCommand: $label | $telemetryText                    "
            $lastCommand = $command
        }

        Start-Sleep -Milliseconds 50
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Write("X")
        Start-Sleep -Milliseconds 50
        $serial.Write("X")
        $serial.Close()
    }

    Write-Host ""
    Write-Host "Stop command sent. COM port closed."
}
