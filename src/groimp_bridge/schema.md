┌─────────────────────┐
│       GroIMP        │
│                     │
│  modello .gsz/RGG   │
│  crescita pianta    │
│                     │
│  GroLink API        │
│  localhost:58081    │
└──────────┬──────────┘
           │ HTTP
           │
           ▼
┌─────────────────────┐
│   Python / GroPy    │
│                     │
│     uv run ...      │
│                     │
│ legge grafo         │
│ esegue RGG          │
│ manda parametri     │
│ riceve risultati    │
└─────────────────────┘