param(
    [Parameter(Mandatory = $true)]
    [string]$DatabasePath,

    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,

    [Parameter(Mandatory = $true)]
    [string]$LineName,

    [Parameter(Mandatory = $true)]
    [string]$HorizontalTableName,

    [Parameter(Mandatory = $true)]
    [string]$OffsetTableName
)

$ErrorActionPreference = 'Stop'
$utf8 = New-Object System.Text.UTF8Encoding($false)
[Console]::OutputEncoding = $utf8
$OutputEncoding = $utf8
$culture = [System.Globalization.CultureInfo]::InvariantCulture

function Format-Double([object]$Value) {
    return ([double]$Value).ToString('R', $culture)
}

function Escape-SqlLiteral([string]$Value) {
    return $Value.Replace("'", "''")
}

function Open-Database([string]$Path) {
    $connection = New-Object -ComObject ADODB.Connection
    $resolvedPath = (Resolve-Path -LiteralPath $Path).Path
    $providers = @('Microsoft.ACE.OLEDB.16.0', 'Microsoft.ACE.OLEDB.12.0', 'Microsoft.Jet.OLEDB.4.0')
    foreach ($provider in $providers) {
        try {
            $connection.Open("Provider=$provider;Data Source=$resolvedPath;Persist Security Info=False;")
            return $connection
        }
        catch {
            if ($connection.State -ne 0) {
                $connection.Close()
            }
        }
    }
    throw "No installed ACE/Jet OLE DB provider could open: $resolvedPath"
}

$resolvedOutput = [System.IO.Path]::GetFullPath($OutputDirectory)
[System.IO.Directory]::CreateDirectory($resolvedOutput) | Out-Null
$connection = Open-Database $DatabasePath

try {
    $horizontalLines = New-Object 'System.Collections.Generic.List[string]'
    $horizontalLines.Add('control_index,name,kind,x,y,radius_m,entry_spiral_length_m,exit_spiral_length_m')

    $escapedHorizontalTable = $HorizontalTableName.Replace(']', ']]')
    $horizontal = New-Object -ComObject ADODB.Recordset
    $horizontal.Open("SELECT * FROM [$escapedHorizontalTable] ORDER BY [ID]", $connection)
    try {
        $rows = @()
        while (-not $horizontal.EOF) {
            if ([string]$horizontal.Fields.Item(1).Value -eq $LineName) {
                $rows += ,@(
                    [string]$horizontal.Fields.Item(2).Value,
                    [double]$horizontal.Fields.Item(3).Value,
                    [double]$horizontal.Fields.Item(4).Value,
                    [double]$horizontal.Fields.Item(6).Value,
                    [double]$horizontal.Fields.Item(5).Value,
                    [double]$horizontal.Fields.Item(7).Value
                )
            }
            $horizontal.MoveNext()
        }
    }
    finally {
        $horizontal.Close()
    }

    if ($rows.Count -ne 22) {
        throw "Expected 22 horizontal control points for '$LineName', got $($rows.Count)"
    }

    for ($index = 0; $index -lt $rows.Count; $index++) {
        $kind = if ($index -eq 0) { 'start' } elseif ($index -eq $rows.Count - 1) { 'end' } else { 'intersection' }
        $row = $rows[$index]
        $horizontalLines.Add((@(
            $index,
            $row[0],
            $kind,
            (Format-Double $row[1]),
            (Format-Double $row[2]),
            (Format-Double $row[3]),
            (Format-Double $row[4]),
            (Format-Double $row[5])
        ) -join ','))
    }

    $offsetLines = New-Object 'System.Collections.Generic.List[string]'
    $offsetLines.Add('point_index,x,y,station_m,property,horizontal_offset_mm,horizontal_offset_m')
    $escapedOffsetTable = $OffsetTableName.Replace(']', ']]')
    $offsets = New-Object -ComObject ADODB.Recordset
    $offsets.Open("SELECT * FROM [$escapedOffsetTable] ORDER BY [ID]", $connection)
    try {
        $index = 0
        while (-not $offsets.EOF) {
            if ([string]$offsets.Fields.Item(1).Value -eq $LineName) {
                $offsetMm = [double]$offsets.Fields.Item(6).Value
                $offsetLines.Add((@(
                    $index,
                    (Format-Double $offsets.Fields.Item(2).Value),
                    (Format-Double $offsets.Fields.Item(3).Value),
                    (Format-Double $offsets.Fields.Item(4).Value),
                    ([int]$offsets.Fields.Item(5).Value),
                    (Format-Double $offsetMm),
                    (Format-Double ($offsetMm / 1000.0))
                ) -join ','))
                $index++
            }
            $offsets.MoveNext()
        }
    }
    finally {
        $offsets.Close()
    }

    if ($index -ne 1569) {
        throw "Expected 1569 offset rows for '$LineName', got $index"
    }

    $horizontalPath = Join-Path $resolvedOutput 'k1413_k1462_horizontal.csv'
    $offsetPath = Join-Path $resolvedOutput 'k1413_k1462_offsets.csv'
    [System.IO.File]::WriteAllLines($horizontalPath, $horizontalLines, $utf8)
    [System.IO.File]::WriteAllLines($offsetPath, $offsetLines, $utf8)

    Write-Output "EXPORTED_HORIZONTAL`t$horizontalPath`t$($rows.Count)"
    Write-Output "EXPORTED_OFFSETS`t$offsetPath`t$index"
}
finally {
    $connection.Close()
}
