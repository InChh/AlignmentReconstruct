import { FileBlob, SpreadsheetFile } from "@oai/artifact-tool";
import fs from "node:fs/promises";
import path from "node:path";

const inputPath = process.argv[2];
const pointsOutputPath = process.argv[3];
if (!inputPath) {
  throw new Error("usage: node inspect_reference_workbook.mjs <input.xlsx>");
}

const input = await FileBlob.load(inputPath);
const workbook = await SpreadsheetFile.importXlsx(input);

if (!pointsOutputPath) {
  const sheets = await workbook.inspect({
    kind: "sheet",
    include: "id,name",
    maxChars: 12000,
  });
  console.log("--- SHEETS ---");
  console.log(sheets.ndjson);

  const overview = await workbook.inspect({
    kind: "workbook,sheet,table",
    maxChars: 30000,
    tableMaxRows: 12,
    tableMaxCols: 12,
    tableMaxCellChars: 120,
  });
  console.log("--- OVERVIEW ---");
  console.log(overview.ndjson);
}

if (pointsOutputPath) {
  const sheet = workbook.worksheets.getItem("Sheet1");
  const values = sheet.getUsedRange(true).values;
  if (values.length !== 1569 || values.some((row) => row.length !== 2)) {
    throw new Error(`expected Sheet1 to contain 1569 rows x 2 columns, got ${values.length} rows`);
  }

  const lines = ["point_index,x,y"];
  values.forEach((row, index) => {
    const x = Number(row[0]);
    const y = Number(row[1]);
    if (!Number.isFinite(x) || !Number.isFinite(y)) {
      throw new Error(`non-finite coordinate at Sheet1 row ${index + 1}`);
    }
    lines.push(`${index},${x.toString()},${y.toString()}`);
  });

  await fs.mkdir(path.dirname(pointsOutputPath), { recursive: true });
  await fs.writeFile(pointsOutputPath, `${lines.join("\n")}\n`, "utf8");
  console.log(`EXPORTED_POINTS\t${pointsOutputPath}\t${values.length}`);
}
