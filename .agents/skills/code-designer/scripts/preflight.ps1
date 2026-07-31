[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectRoot
)

$ErrorActionPreference = "Stop"
$root = (Resolve-Path -LiteralPath $ProjectRoot).Path
$requiredFiles = @(
    "AGENTS.md",
    "MEMORY.md",
    "memory/rules.md",
    "memory/workflow.md",
    "memory/project.md",
    "memory/apply.md",
    "memory/todo.md"
)

Push-Location -LiteralPath $root
try {
    if(-not (Test-Path -LiteralPath ".git")){
        throw "ProjectRoot is not a Git repository: $root"
    }

    $missingFiles = @($requiredFiles | Where-Object {
        -not (Test-Path -LiteralPath (Join-Path $root $_))
    })
    if($missingFiles.Count -gt 0){
        throw "Missing required project files: $($missingFiles -join ', ')"
    }

    Write-Output "Project root: $root"
    Write-Output "Required project memory: OK"
    Write-Output "Existing worktree changes:"
    & git status --short
    if($LASTEXITCODE -ne 0){
        throw "git status failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
