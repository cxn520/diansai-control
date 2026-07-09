param(
    [Parameter(Position = 0)]
    [string]$Message = "update"
)

$ErrorActionPreference = "Stop"

function Get-GitCommand {
    $git = Get-Command git -ErrorAction SilentlyContinue
    if ($git) {
        return $git.Source
    }

    $fallback = "D:\Program Files\Git\cmd\git.exe"
    if (Test-Path $fallback) {
        return $fallback
    }

    throw "git was not found. Install Git for Windows or add git to PATH."
}

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $repoRoot

$gitCmd = Get-GitCommand
$timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss"
$commitMessage = "$Message [$timestamp]"

& $gitCmd rev-parse --is-inside-work-tree | Out-Null

$status = & $gitCmd status --porcelain
if (-not $status) {
    Write-Host "No changes to commit."
    exit 0
}

& $gitCmd add .
& $gitCmd commit -m $commitMessage
& $gitCmd push

Write-Host "Pushed successfully: $commitMessage"
