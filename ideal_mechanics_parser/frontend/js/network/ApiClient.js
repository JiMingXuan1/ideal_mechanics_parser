export class ApiClient {
  constructor(endpointBase = 'http://localhost:8000') {
    this.base = endpointBase;
  }

  async solve(topology) {
    const resp = await fetch(`${this.base}/solve`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(topology),
    });
    if (!resp.ok) {
      let msg = `HTTP ${resp.status}`;
      try { const e = await resp.json(); if (e.error) msg = e.error; } catch {}
      throw new Error(msg);
    }
    const data = await resp.json();
    if (window.__logger) window.__logger.logNetwork(topology, data);
    return data;
  }

  streamSolve(topology, onChunk) {
    return new Promise((resolve, reject) => {
      fetch(`${this.base}/solve/stream`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(topology),
      }).then(async (resp) => {
        if (!resp.ok) {
          let msg = `HTTP ${resp.status}`;
          try { const e = await resp.json(); if (e.error) msg = e.error; } catch {}
          reject(new Error(msg));
          return;
        }
        const reader = resp.body.getReader();
        const decoder = new TextDecoder();
        let buf = '';
        while (true) {
          const { done, value } = await reader.read();
          if (done) break;
          buf += decoder.decode(value, { stream: true });
          const lines = buf.split('\n');
          buf = lines.pop();
          for (const line of lines) {
            if (line.startsWith('data: ')) {
              const chunk = JSON.parse(line.slice(6));
              if (window.__logger && 'q' in chunk) window.__logger.logNetwork(topology, chunk);
              onChunk(chunk);
              if (chunk.complete) resolve();
            }
          }
        }
      }).catch(reject);
    });
  }
}
