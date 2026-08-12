# Arquivo histórico: relatório inicial da PoC

> **Este documento foi substituído.** Ele é mantido apenas como registro histórico da fase inicial do projeto e não descreve a implementação atual, nem deve ser citado como benchmark ou comprovação de desempenho.

A implementação, os contratos de dados, as unidades físicas e as limitações que valem para a versão pública estão documentados em inglês em [Architecture Note](architecture.md) e no [README](../README.md).

## O que mudou desde o relatório inicial

| Tema | Estado atual documentado |
|---|---|
| Projeção RGB-D | Usa profundidade métrica ao longo do raio de câmera normalizado. |
| Travessia | Usa DDA discreto na resolução das folhas, em CPU. |
| Doppler | Distingue a amostragem acústica clássica do helper óptico relativístico longitudinal. |
| Validação | Build CMake, CTest e configuração opcional com sanitizadores. |
| Alegações de desempenho | Nenhum benchmark de GPU, tempo real ou produção é reivindicado. |

O texto original não é distribuído como especificação técnica porque continha afirmações e estimativas que não foram validadas nesta versão da PoC.
