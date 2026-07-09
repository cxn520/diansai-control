param(
    [switch]$PreviewOnly
)

$ErrorActionPreference = 'Stop'

Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes

$source = @"
using System;
using System.Runtime.InteropServices;
using System.Text;

public static class DesktopListViewApi
{
    private const int LVM_FIRST = 0x1000;
    private const int LVM_GETITEMCOUNT = LVM_FIRST + 4;
    private const int LVM_GETITEMPOSITION = LVM_FIRST + 16;
    private const int LVM_SETITEMPOSITION = LVM_FIRST + 15;
    private const int LVM_GETITEMSPACING = LVM_FIRST + 51;
    private const int LVM_GETITEMTEXTW = LVM_FIRST + 115;

    private const uint PROCESS_VM_OPERATION = 0x0008;
    private const uint PROCESS_VM_READ = 0x0010;
    private const uint PROCESS_VM_WRITE = 0x0020;
    private const uint PROCESS_QUERY_INFORMATION = 0x0400;

    private const uint MEM_COMMIT = 0x1000;
    private const uint MEM_RESERVE = 0x2000;
    private const uint MEM_RELEASE = 0x8000;
    private const uint PAGE_READWRITE = 0x04;

    private static IntPtr _cachedHandle = IntPtr.Zero;
    private static IntPtr _cachedProcess = IntPtr.Zero;

    [StructLayout(LayoutKind.Sequential)]
    private struct RECT
    {
        public int Left;
        public int Top;
        public int Right;
        public int Bottom;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct POINT
    {
        public int X;
        public int Y;
    }

    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
    private struct LVITEM
    {
        public uint mask;
        public int iItem;
        public int iSubItem;
        public uint state;
        public uint stateMask;
        public IntPtr pszText;
        public int cchText;
        public int iImage;
        public IntPtr lParam;
        public int iIndent;
        public int iGroupId;
        public uint cColumns;
        public IntPtr puColumns;
        public IntPtr piColFmt;
        public int iGroup;
    }

    [DllImport("user32.dll", SetLastError = true)]
    private static extern IntPtr SendMessage(IntPtr hWnd, int msg, IntPtr wParam, IntPtr lParam);

    [DllImport("user32.dll", SetLastError = true)]
    private static extern bool GetClientRect(IntPtr hWnd, out RECT rect);

    [DllImport("user32.dll", SetLastError = true)]
    private static extern uint GetWindowThreadProcessId(IntPtr hWnd, out uint processId);

    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern IntPtr OpenProcess(uint access, bool inheritHandle, uint processId);

    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool CloseHandle(IntPtr hObject);

    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern IntPtr VirtualAllocEx(IntPtr hProcess, IntPtr address, UIntPtr size, uint allocationType, uint protect);

    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool VirtualFreeEx(IntPtr hProcess, IntPtr address, UIntPtr size, uint freeType);

    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool ReadProcessMemory(IntPtr hProcess, IntPtr address, byte[] buffer, int size, out IntPtr bytesRead);

    [DllImport("kernel32.dll", SetLastError = true)]
    private static extern bool WriteProcessMemory(IntPtr hProcess, IntPtr address, byte[] buffer, int size, out IntPtr bytesWritten);

    public static int GetItemCount(IntPtr handle)
    {
        return SendMessage(handle, LVM_GETITEMCOUNT, IntPtr.Zero, IntPtr.Zero).ToInt32();
    }

    public static int[] GetClientSize(IntPtr handle)
    {
        RECT rect;
        if (!GetClientRect(handle, out rect))
        {
            throw new InvalidOperationException("Unable to get the desktop client size.");
        }

        return new[] { rect.Right - rect.Left, rect.Bottom - rect.Top };
    }

    public static int[] GetItemSpacing(IntPtr handle)
    {
        int packed = SendMessage(handle, LVM_GETITEMSPACING, IntPtr.Zero, IntPtr.Zero).ToInt32();
        return new[] { packed & 0xFFFF, (packed >> 16) & 0xFFFF };
    }

    public static int[] GetItemPosition(IntPtr handle, int index)
    {
        IntPtr process = OpenDesktopProcess(handle);
        IntPtr remotePoint = VirtualAllocEx(process, IntPtr.Zero, (UIntPtr)Marshal.SizeOf<POINT>(), MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        if (remotePoint == IntPtr.Zero)
        {
            throw new InvalidOperationException("Unable to allocate remote memory for icon position.");
        }

        try
        {
            SendMessage(handle, LVM_GETITEMPOSITION, (IntPtr)index, remotePoint);
            byte[] buffer = new byte[Marshal.SizeOf<POINT>()];
            IntPtr bytesRead;
            if (!ReadProcessMemory(process, remotePoint, buffer, buffer.Length, out bytesRead))
            {
                throw new InvalidOperationException("Unable to read icon position.");
            }

            GCHandle pinned = GCHandle.Alloc(buffer, GCHandleType.Pinned);
            try
            {
                POINT point = Marshal.PtrToStructure<POINT>(pinned.AddrOfPinnedObject());
                return new[] { point.X, point.Y };
            }
            finally
            {
                pinned.Free();
            }
        }
        finally
        {
            VirtualFreeEx(process, remotePoint, UIntPtr.Zero, MEM_RELEASE);
        }
    }

    public static string GetItemText(IntPtr handle, int index)
    {
        IntPtr process = OpenDesktopProcess(handle);
        int textCapacity = 512;
        int textBytes = textCapacity * 2;
        IntPtr remoteText = VirtualAllocEx(process, IntPtr.Zero, (UIntPtr)textBytes, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        IntPtr remoteItem = VirtualAllocEx(process, IntPtr.Zero, (UIntPtr)Marshal.SizeOf<LVITEM>(), MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
        if (remoteText == IntPtr.Zero || remoteItem == IntPtr.Zero)
        {
            if (remoteText != IntPtr.Zero) VirtualFreeEx(process, remoteText, UIntPtr.Zero, MEM_RELEASE);
            if (remoteItem != IntPtr.Zero) VirtualFreeEx(process, remoteItem, UIntPtr.Zero, MEM_RELEASE);
            throw new InvalidOperationException("Unable to allocate remote memory for icon text.");
        }

        try
        {
            LVITEM item = new LVITEM
            {
                iSubItem = 0,
                cchText = textCapacity,
                pszText = remoteText
            };

            int itemSize = Marshal.SizeOf<LVITEM>();
            byte[] itemBytes = new byte[itemSize];
            IntPtr localItem = Marshal.AllocHGlobal(itemSize);
            try
            {
                Marshal.StructureToPtr(item, localItem, false);
                Marshal.Copy(localItem, itemBytes, 0, itemSize);
            }
            finally
            {
                Marshal.FreeHGlobal(localItem);
            }

            IntPtr bytesWritten;
            if (!WriteProcessMemory(process, remoteItem, itemBytes, itemBytes.Length, out bytesWritten))
            {
                throw new InvalidOperationException("Unable to write LVITEM data.");
            }

            SendMessage(handle, LVM_GETITEMTEXTW, (IntPtr)index, remoteItem);

            byte[] textBuffer = new byte[textBytes];
            IntPtr bytesRead;
            if (!ReadProcessMemory(process, remoteText, textBuffer, textBuffer.Length, out bytesRead))
            {
                throw new InvalidOperationException("Unable to read icon text.");
            }

            return Encoding.Unicode.GetString(textBuffer).TrimEnd('\0');
        }
        finally
        {
            VirtualFreeEx(process, remoteText, UIntPtr.Zero, MEM_RELEASE);
            VirtualFreeEx(process, remoteItem, UIntPtr.Zero, MEM_RELEASE);
        }
    }

    public static void SetItemPosition(IntPtr handle, int index, int x, int y)
    {
        int packed = (y << 16) | (x & 0xFFFF);
        SendMessage(handle, LVM_SETITEMPOSITION, (IntPtr)index, (IntPtr)packed);
    }

    public static void ReleaseCache()
    {
        if (_cachedProcess != IntPtr.Zero)
        {
            CloseHandle(_cachedProcess);
            _cachedProcess = IntPtr.Zero;
            _cachedHandle = IntPtr.Zero;
        }
    }

    private static IntPtr OpenDesktopProcess(IntPtr handle)
    {
        if (_cachedHandle == handle && _cachedProcess != IntPtr.Zero)
        {
            return _cachedProcess;
        }

        ReleaseCache();

        uint processId;
        GetWindowThreadProcessId(handle, out processId);
        IntPtr process = OpenProcess(PROCESS_VM_OPERATION | PROCESS_VM_READ | PROCESS_VM_WRITE | PROCESS_QUERY_INFORMATION, false, processId);
        if (process == IntPtr.Zero)
        {
            throw new InvalidOperationException("Unable to open the Explorer process.");
        }

        _cachedHandle = handle;
        _cachedProcess = process;
        return process;
    }
}
"@

Add-Type -TypeDefinition $source -Language CSharp

function Get-DesktopListViewHandle {
    $root = [System.Windows.Automation.AutomationElement]::RootElement
    $condition = New-Object System.Windows.Automation.PropertyCondition(
        [System.Windows.Automation.AutomationElement]::ClassNameProperty,
        'SysListView32'
    )
    $desktop = $root.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $condition)
    if (-not $desktop) {
        throw 'Desktop icon view (SysListView32) was not found. Show the desktop and run the script again.'
    }

    return [IntPtr]::new($desktop.Current.NativeWindowHandle)
}

function Test-IsWorkSoftware {
    param(
        [string]$Name
    )

    $workPatterns = @(
        'codex',
        'codebuddy',
        'visual studio',
        'visual studio code',
        'vscode',
        'keil',
        'hbuilder',
        'mqttx',
        'ccs',
        'cc switch',
        'canmv',
        'renesas',
        'e2 studio',
        'smart configurator',
        'uniflash',
        'sysconfig',
        'system configuration',
        'ti system configuration',
        'flymcu',
        'stc-isp',
        'eda',
        'push.bat'
    )

    $candidate = $Name.ToLowerInvariant()
    foreach ($pattern in $workPatterns) {
        if ($candidate -like "*$pattern*") {
            return $true
        }
    }

    return $false
}

function Get-DesktopEntries {
    $desktopPath = [Environment]::GetFolderPath('Desktop')
    $entriesByName = @{}

    Get-ChildItem -LiteralPath $desktopPath -Force |
        Where-Object { $_.Name -ne 'desktop.ini' } |
        ForEach-Object {
            $entriesByName[$_.Name] = [PSCustomObject]@{
                Name = $_.Name
                FullName = $_.FullName
                IsWorkSoftware = Test-IsWorkSoftware -Name $_.Name
            }
        }

    return $entriesByName
}

function Get-DesktopIconState {
    $handle = Get-DesktopListViewHandle
    $entriesByName = Get-DesktopEntries
    $count = [DesktopListViewApi]::GetItemCount($handle)
    $spacing = [DesktopListViewApi]::GetItemSpacing($handle)
    $client = [DesktopListViewApi]::GetClientSize($handle)
    $icons = New-Object System.Collections.Generic.List[object]

    for ($index = 0; $index -lt $count; $index++) {
        $name = [DesktopListViewApi]::GetItemText($handle, $index)
        if ([string]::IsNullOrWhiteSpace($name)) {
            continue
        }

        $entry = $entriesByName[$name]
        $icons.Add([PSCustomObject]@{
            Index = $index
            Name = $name
            IsWorkSoftware = [bool]($entry -and $entry.IsWorkSoftware)
        })
    }

    return [PSCustomObject]@{
        Handle = $handle
        ClientWidth = $client[0]
        ClientHeight = $client[1]
        SpacingX = [Math]::Max($spacing[0], 90)
        SpacingY = [Math]::Max($spacing[1], 90)
        Icons = $icons
    }
}

function Get-TargetPositions {
    param(
        [Parameter(Mandatory)]
        $State
    )

    $marginLeft = 20
    $marginTop = 20
    $marginRight = 20
    $usableHeight = [Math]::Max($State.ClientHeight - ($marginTop * 2), $State.SpacingY)
    $rowsPerColumn = [Math]::Max([Math]::Floor($usableHeight / $State.SpacingY), 1)

    $leftIcons = $State.Icons | Where-Object { -not $_.IsWorkSoftware } | Sort-Object Name
    $rightIcons = $State.Icons | Where-Object { $_.IsWorkSoftware } | Sort-Object Name

    $layout = New-Object System.Collections.Generic.List[object]

    for ($i = 0; $i -lt $leftIcons.Count; $i++) {
        $col = [Math]::Floor($i / $rowsPerColumn)
        $row = $i % $rowsPerColumn
        $layout.Add([PSCustomObject]@{
            Index = $leftIcons[$i].Index
            Name = $leftIcons[$i].Name
            Group = 'Left'
            X = $marginLeft + ($col * $State.SpacingX)
            Y = $marginTop + ($row * $State.SpacingY)
        })
    }

    for ($i = 0; $i -lt $rightIcons.Count; $i++) {
        $col = [Math]::Floor($i / $rowsPerColumn)
        $row = $i % $rowsPerColumn
        $x = $State.ClientWidth - $marginRight - $State.SpacingX - ($col * $State.SpacingX)
        $layout.Add([PSCustomObject]@{
            Index = $rightIcons[$i].Index
            Name = $rightIcons[$i].Name
            Group = 'Right'
            X = [Math]::Max($x, $marginLeft)
            Y = $marginTop + ($row * $State.SpacingY)
        })
    }

    return $layout | Sort-Object Group, Y, X
}

try {
    $state = Get-DesktopIconState
    $layout = Get-TargetPositions -State $state

    if ($PreviewOnly) {
        $layout | Select-Object Group, Name, X, Y | Format-Table -AutoSize
        return
    }

    foreach ($item in $layout) {
        [DesktopListViewApi]::SetItemPosition($state.Handle, $item.Index, $item.X, $item.Y)
    }

    Write-Host 'Desktop icons arranged: work apps on the right, other items on the left.'
}
finally {
    [DesktopListViewApi]::ReleaseCache()
}
