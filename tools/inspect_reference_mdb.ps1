param(
    [Parameter(Mandatory = $true)]
    [string]$DatabasePath,

    [string]$TableName = '',

    [int]$MaxRows = 10
)

$ErrorActionPreference = 'Stop'
$utf8 = New-Object System.Text.UTF8Encoding($false)
[Console]::OutputEncoding = $utf8
$OutputEncoding = $utf8

$resolvedPath = (Resolve-Path -LiteralPath $DatabasePath).Path
$connection = New-Object -ComObject ADODB.Connection
$providers = @(
    'Microsoft.ACE.OLEDB.16.0',
    'Microsoft.ACE.OLEDB.12.0',
    'Microsoft.Jet.OLEDB.4.0'
)

$openedProvider = $null
foreach ($provider in $providers) {
    try {
        $connection.Open("Provider=$provider;Data Source=$resolvedPath;Persist Security Info=False;")
        $openedProvider = $provider
        break
    }
    catch {
        if ($connection.State -ne 0) {
            $connection.Close()
        }
    }
}

if (-not $openedProvider) {
    throw "No installed ACE/Jet OLE DB provider could open: $resolvedPath"
}

try {
    Write-Output "PROVIDER`t$openedProvider"
    Write-Output "DATABASE`t$resolvedPath"

    $schemaTables = 20
    $recordset = $connection.OpenSchema($schemaTables)
    try {
        while (-not $recordset.EOF) {
            $tableType = [string]$recordset.Fields.Item('TABLE_TYPE').Value
            if ($tableType -eq 'TABLE') {
                $schemaTableName = [string]$recordset.Fields.Item('TABLE_NAME').Value
                Write-Output "TABLE`t$schemaTableName"
            }
            $recordset.MoveNext()
        }
    }
    finally {
        $recordset.Close()
    }

    if ($TableName) {
        $escapedTableName = $TableName.Replace(']', ']]')

        $countSet = New-Object -ComObject ADODB.Recordset
        $countSet.Open("SELECT COUNT(*) AS RowCount FROM [$escapedTableName]", $connection)
        try {
            Write-Output "ROW_COUNT`t$($countSet.Fields.Item('RowCount').Value)"
        }
        finally {
            $countSet.Close()
        }

        $dataSet = New-Object -ComObject ADODB.Recordset
        $dataSet.Open("SELECT TOP $MaxRows * FROM [$escapedTableName]", $connection)
        try {
            $columnNames = @()
            for ($columnIndex = 0; $columnIndex -lt $dataSet.Fields.Count; $columnIndex++) {
                $columnNames += [string]$dataSet.Fields.Item($columnIndex).Name
            }
            Write-Output ("COLUMNS`t" + ($columnNames -join "`t"))

            while (-not $dataSet.EOF) {
                $values = @()
                for ($columnIndex = 0; $columnIndex -lt $dataSet.Fields.Count; $columnIndex++) {
                    $value = $dataSet.Fields.Item($columnIndex).Value
                    if ($null -eq $value -or $value -is [System.DBNull]) {
                        $values += ''
                    }
                    else {
                        $values += ([string]$value).Replace("`t", ' ').Replace("`r", ' ').Replace("`n", ' ')
                    }
                }
                Write-Output ("ROW`t" + ($values -join "`t"))
                $dataSet.MoveNext()
            }
        }
        finally {
            $dataSet.Close()
        }
    }
}
finally {
    $connection.Close()
}
