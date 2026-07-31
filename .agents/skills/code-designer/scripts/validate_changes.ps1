[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectRoot,

    [Parameter(Mandatory = $true)]
    [ValidateNotNullOrEmpty()]
    [string[]]$Paths
)

$ErrorActionPreference = "Stop"
$root = (Resolve-Path -LiteralPath $ProjectRoot).Path
$separatorChars = [char[]]@(
    [System.IO.Path]::DirectorySeparatorChar,
    [System.IO.Path]::AltDirectorySeparatorChar
)
$rootPrefix = $root.TrimEnd($separatorChars) + [System.IO.Path]::DirectorySeparatorChar
$normalizedPaths = @()

foreach($path in $Paths){
    $pathValue = $path.Trim()
    if([string]::IsNullOrWhiteSpace($pathValue)){
        throw "Validation paths must not be empty"
    }
    if([System.IO.Path]::IsPathRooted($pathValue)){
        throw "Validation paths must be relative: $pathValue"
    }

    $gitPath = $pathValue.Replace("\", "/")
    while($gitPath.StartsWith("./")){
        $gitPath = $gitPath.Substring(2)
    }
    if(($gitPath -eq ".") -or ($gitPath.Split("/") -contains "..")){
        throw "Validation path is too broad or escapes the project root: $pathValue"
    }

    $fullPath = [System.IO.Path]::GetFullPath((Join-Path $root $gitPath))
    if(-not $fullPath.StartsWith(
        $rootPrefix,
        [System.StringComparison]::OrdinalIgnoreCase
    )){
        throw "Validation path escapes the project root: $pathValue"
    }
    $normalizedPaths += $gitPath
}

$normalizedPaths = @($normalizedPaths | Sort-Object -Unique)

Push-Location -LiteralPath $root
try {
    if(-not (Test-Path -LiteralPath ".git")){
        throw "ProjectRoot is not a Git repository: $root"
    }

    & git diff --check -- $normalizedPaths
    $worktreeCheck = $LASTEXITCODE
    & git diff --cached --check -- $normalizedPaths
    $indexCheck = $LASTEXITCODE
    if(($worktreeCheck -ne 0) -or ($indexCheck -ne 0)){
        throw "git diff --check failed for the requested paths"
    }

    $worktreePaths = @(& git diff --name-only -- $normalizedPaths)
    $indexPaths = @(& git diff --cached --name-only -- $normalizedPaths)
    $untrackedPaths = @(& git ls-files --others --exclude-standard -- $normalizedPaths)
    $changedPaths = @($worktreePaths + $indexPaths + $untrackedPaths |
        Where-Object { $_ } |
        Sort-Object -Unique)

    Write-Output "Validation scope:"
    $normalizedPaths | ForEach-Object { Write-Output "  $_" }
    Write-Output "Changed paths in scope: $($changedPaths.Count)"
    $changedPaths | ForEach-Object { Write-Output "  $_" }

    $embeddedPaths = @($changedPaths | Where-Object {
        $_ -match '^hero_(down|up)/.*\.(c|h|s|S|cpp|hpp)$'
    })
    Write-Output "Embedded source paths: $($embeddedPaths.Count)"

    foreach($path in $embeddedPaths){
        if($path.StartsWith("hero_down/")){
            $counterpart = "hero_up/" + $path.Substring("hero_down/".Length)
        }
        elseif($path.StartsWith("hero_up/")){
            $counterpart = "hero_down/" + $path.Substring("hero_up/".Length)
        }
        else {
            continue
        }

        if((Test-Path -LiteralPath $counterpart) -and
            ($changedPaths -notcontains $counterpart)){
            Write-Warning "Review the unchanged counterpart: $counterpart"
        }
    }

    $worktreeAddedPaths = @(& git diff --name-only --diff-filter=A -- $normalizedPaths)
    $indexAddedPaths = @(& git diff --cached --name-only --diff-filter=A -- $normalizedPaths)
    $newCFiles = @($worktreeAddedPaths + $indexAddedPaths + $untrackedPaths |
        Where-Object { $_ -match '^hero_(down|up)/.*\.c$' } |
        Sort-Object -Unique)
    foreach($path in $newCFiles){
        Write-Warning "New C file needs Keil and compile database registration: $path"
    }

    Write-Output "Scoped diff checks: OK"
}
finally {
    Pop-Location
}
