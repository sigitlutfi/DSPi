#!/usr/bin/env node
/*
 * server.mjs — minimal dependency-free static file server for the DSPi
 * WebHID console.  Run from the repo root:
 *
 *   node server.mjs          # → http://localhost:8000/web_hid_console_v2.html
 *   node server.mjs 9000     # custom port
 *
 * Why a server at all: the console's RTA panel uses getUserMedia(), which
 * only works in a secure context.  file:// is not one, but http://localhost
 * is — so the page must be served over HTTP.
 */
import { createServer } from "node:http";
import { readFile, stat } from "node:fs/promises";
import { extname, join, normalize } from "node:path";
import { fileURLToPath } from "node:url";

const root = fileURLToPath(new URL(".", import.meta.url));
const port = Number(process.argv[2]) || 8000;

const MIME = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".md": "text/markdown; charset=utf-8",
  ".txt": "text/plain; charset=utf-8",
  ".svg": "image/svg+xml",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".jpeg": "image/jpeg",
  ".ico": "image/x-icon",
};

createServer(async (req, res) => {
  try {
    const url = new URL(req.url, "http://localhost");
    let pathname = decodeURIComponent(url.pathname);
    if (pathname === "/") pathname = "/web_hid_console_v2.html";
    const file = normalize(join(root, pathname));
    if (!file.startsWith(root)) {
      res.writeHead(403).end("Forbidden");
      return;
    }
    const info = await stat(file);
    if (info.isDirectory()) {
      res.writeHead(404).end("Not found");
      return;
    }
    const body = await readFile(file);
    res.writeHead(200, {
      "Content-Type": MIME[extname(file).toLowerCase()] || "application/octet-stream",
      "Content-Length": body.length,
      "Cache-Control": "no-store",
    });
    res.end(body);
  } catch {
    res.writeHead(404).end("Not found");
  }
}).listen(port, () => {
  console.log(`DSPi console server: http://localhost:${port}/web_hid_console_v2.html`);
});
