param(
    [Parameter(Position = 0)]
    [string]$Message
)

$ErrorActionPreference = "Stop"
# PowerShell 7 can turn an expected non-zero exit from a native command into a
# terminating error. Invoke-Git below handles real Git failures explicitly.
if ($PSVersionTable.PSVersion.Major -ge 7) {
    $PSNativeCommandUseErrorActionPreference = $false
}

function Test-LocalPort {
    param([int]$Port)

    $client = [System.Net.Sockets.TcpClient]::new()
    try {
        $connect = $client.BeginConnect("127.0.0.1", $Port, $null, $null)
        if (-not $connect.AsyncWaitHandle.WaitOne(300, $false)) {
            return $false
        }

        $client.EndConnect($connect)
        return $true
    }
    catch {
        return $false
    }
    finally {
        $client.Close()
    }
}

function Enable-LocalProxy {
    foreach ($port in @(7897, 7890, 10809, 10808)) {
        if (Test-LocalPort -Port $port) {
            $proxy = "http://127.0.0.1:$port"
            $env:HTTP_PROXY = $proxy
            $env:HTTPS_PROXY = $proxy
            $env:ALL_PROXY = $proxy
            Write-Host "Using local proxy: $proxy"
            return
        }
    }

    Remove-Item Env:HTTP_PROXY -ErrorAction SilentlyContinue
    Remove-Item Env:HTTPS_PROXY -ErrorAction SilentlyContinue
    Remove-Item Env:ALL_PROXY -ErrorAction SilentlyContinue
    Write-Host "No local proxy port found. Continuing without proxy."
}

function Invoke-Git {
    param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Arguments)

    & $script:gitCmd @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Git command failed: git $($Arguments -join ' ')"
    }
}

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location -LiteralPath $repoRoot

$git = Get-Command git -ErrorAction SilentlyContinue
if (-not $git) {
    throw "Git was not found. Install Git for Windows or add git to PATH."
}
$script:gitCmd = $git.Source

# This check is intentionally against this directory only: never use a parent Git repository.
if (-not (Test-Path -LiteralPath (Join-Path $repoRoot ".git"))) {
    Write-Host "Initializing a Git repository in this project..."
    Invoke-Git -Arguments @("init", "-b", "main")
}

$actualRoot = (& $script:gitCmd rev-parse --show-toplevel).Trim()
if ($LASTEXITCODE -ne 0 -or ([IO.Path]::GetFullPath($actualRoot) -ne [IO.Path]::GetFullPath($repoRoot))) {
    throw "The Git repository root must be this project directory: $repoRoot"
}

$userName = (& $script:gitCmd config --get user.name).Trim()
if ($LASTEXITCODE -ne 0 -or -not $userName) {
    Invoke-Git -Arguments @("config", "user.name", "23169")
}

$userEmail = (& $script:gitCmd config --get user.email).Trim()
if ($LASTEXITCODE -ne 0 -or -not $userEmail) {
    Invoke-Git -Arguments @("config", "user.email", "23169@users.noreply.github.com")
}

$remoteNames = @(& $script:gitCmd remote)
if ($LASTEXITCODE -ne 0) {
    throw "Unable to read Git remotes."
}

$origin = ""
if ($remoteNames -contains "origin") {
    $origin = (& $script:gitCmd remote get-url origin).Trim()
    if ($LASTEXITCODE -ne 0) {
        throw "Unable to read the origin remote URL."
    }
}

if (-not $origin) {
    $origin = Read-Host "Paste the GitHub repository URL (for example https://github.com/account/repository.git)"
    if ([string]::IsNullOrWhiteSpace($origin)) {
        throw "No GitHub repository URL was provided. Create an empty repository on GitHub, then run this script again."
    }
    Invoke-Git -Arguments @("remote", "add", "origin", $origin.Trim())
}

Enable-LocalProxy

if ([string]::IsNullOrWhiteSpace($Message)) {
    $Message = Read-Host "Commit message (press Enter for update)"
}
if ([string]::IsNullOrWhiteSpace($Message)) {
    $Message = "update"
}

$status = (& $script:gitCmd status --porcelain)
if ($LASTEXITCODE -ne 0) {
    throw "Unable to read Git status."
}

if ($status) {
    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
    Invoke-Git -Arguments @("add", "-A")
    Invoke-Git -Arguments @("commit", "-m", "$Message [$timestamp]")
}
else {
    Write-Host "No file changes to commit; pushing existing local commits if any."
}

$branch = (& $script:gitCmd branch --show-current).Trim()
if ([string]::IsNullOrWhiteSpace($branch)) {
    throw "No current branch exists. Add at least one project file, then run this script again."
}

$upstream = (& $script:gitCmd for-each-ref --format="%(upstream:short)" "refs/heads/$branch").Trim()
if ($LASTEXITCODE -ne 0) {
    throw "Unable to read the branch upstream."
}
if ($upstream) {
    Invoke-Git -Arguments @("push")
}
else {
    Invoke-Git -Arguments @("push", "-u", "origin", $branch)
}

Write-Host "Pushed successfully."
