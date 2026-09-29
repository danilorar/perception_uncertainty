# Portable pinned downloads only. Does not change PATH, registry, or system packages.
$ErrorActionPreference = 'Stop'
$labRoot = Split-Path -Parent $PSScriptRoot
$items = @(
    @{
        Name = 'esmini-demo_Windows-v3.8.1.zip'
        Url = 'https://github.com/esmini/esmini/releases/download/v3.8.1/esmini-demo_Windows.zip'
        Sha256 = '2983be14adba66e5cc22f52714c82e1c06c766facf6864b59c911a1860b9a319'
        Destination = 'runtime'
        Expected = 'runtime/esmini-demo/bin/esmini.exe'
    },
    @{
        Name = 'OSC-NCAP-scenarios-15365d18.zip'
        Url = 'https://api.github.com/repos/vectorgrp/OSC-NCAP-scenarios/zipball/15365d18bd7d1d6aff46c75938eddaf4325ac8f3'
        # Locally recorded archive hash, not a publisher-signed checksum.
        Sha256 = '11134e3450b8351a740b8ace16ac635ea99404e3c1756ed72e6e4b0a93295c99'
        Destination = 'sources'
        Expected = 'sources/vectorgrp-OSC-NCAP-scenarios-15365d1/README.md'
    }
)
New-Item -ItemType Directory -Force -Path (Join-Path $labRoot 'downloads'), (Join-Path $labRoot 'evidence') | Out-Null
$receipt = foreach ($item in $items) {
    $archive = Join-Path $labRoot ('downloads/' + $item.Name)
    if (-not (Test-Path -LiteralPath $archive)) {
        $downloadLog = Join-Path $labRoot ('evidence/setup-download-' + $item.Name + '.log')
        & curl.exe --fail --location --retry 2 --connect-timeout 30 --max-time 600 --output $archive $item.Url 2> $downloadLog
        if ($LASTEXITCODE -ne 0) { throw "Download failed: $($item.Name); see $downloadLog" }
    }
    $actualHash = (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash.ToLower()
    if ($actualHash -ne $item.Sha256) { throw "Checksum mismatch: $($item.Name); no files extracted" }
    Unblock-File -LiteralPath $archive
    $expectedFile = Join-Path $labRoot $item.Expected
    if (-not (Test-Path -LiteralPath $expectedFile)) {
        Expand-Archive -LiteralPath $archive -DestinationPath (Join-Path $labRoot $item.Destination)
    }
    [ordered]@{name=$item.Name; url=$item.Url; sha256=$actualHash; bytes=(Get-Item -LiteralPath $archive).Length; expected_file_present=(Test-Path -LiteralPath $expectedFile)}
}
[ordered]@{captured_at=(Get-Date -Format o); installed_system_dependencies=@(); downloads=@($receipt)} |
    ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $labRoot 'evidence/setup-receipt.json') -Encoding utf8
$receipt | ConvertTo-Json -Depth 4
