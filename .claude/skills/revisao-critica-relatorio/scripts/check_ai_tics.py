#!/usr/bin/env python3
"""Detector determinístico de idiossincrasias de texto gerado por IA e lacunas
de rigor em documentos científicos LaTeX/Markdown deste projeto.

Roda em segundos, sem gastar contexto de LLM. Reporta contagens e linhas,
não decide sozinho o que cortar — julgamento fica com quem lê o relatório.

Uso: python3 check_ai_tics.py <arquivo.tex|arquivo.md>
"""
import re
import sys
from pathlib import Path


ADJETIVOS_VAZIOS = [
    "robusto", "robusta", "eficiente", "eficientes", "significativo", "significativa",
    "significativos", "significativas", "fundamental", "crucial", "essencial",
    "otimizado", "otimizada", "completo", "completa", "fielmente", "consistentemente",
    "natural", "confiável", "estável",
]

CONECTIVOS_CLICHE = [
    "além disso", "dessa forma", "desse modo", "nesse sentido", "nesta perspectiva",
    "vale ressaltar que", "é importante notar que", "é importante destacar que",
    "cabe destacar que", "cumpre destacar que", "não obstante", "ademais",
]

TRANSICOES_VAZIAS = [
    "isso significa que", "em outras palavras", "dito de outra forma",
    "ou seja, isso implica", "posto de outra maneira",
]

ADJETIVOS_COORDENADOS_SUSPEITOS = [
    (r"robust\w+ e eficient\w+", "robusto e eficiente"),
    (r"clar\w+ e concis\w+", "claro e conciso"),
    (r"rigoros\w+ e reproduz[íi]v\w+", "rigoroso e reprodutível"),
    (r"simples e direto", "simples e direto"),
    (r"amplo e abrangente", "amplo e abrangente"),
]


def contar(pattern: str, texto: str, flags=0) -> list[tuple[int, str]]:
    achados = []
    for i, linha in enumerate(texto.split("\n"), 1):
        if re.search(pattern, linha, flags):
            achados.append((i, linha.strip()))
    return achados


def check_travessao(texto: str):
    achados = contar(r"---", texto)
    print(f"\n=== Travessões (---): {len(achados)} ocorrências ===")
    print("Regra do projeto: sem travessão em prosa. Trocar por vírgula, ponto,")
    print("dois-pontos ou reestruturar a frase (nunca por hífen simples).")
    for ln, txt in achados[:15]:
        print(f"  L{ln}: {txt[:100]}")
    if len(achados) > 15:
        print(f"  ... e mais {len(achados) - 15}")


def check_antitese(texto: str):
    pattern = (
        r"não apenas .+?,? mas|não só .+?,? mas|, não .{3,40}(?:,|\.|;)"
        r"|não é apenas .{3,40},? é "
    )
    achados = contar(pattern, texto, re.IGNORECASE)
    print(f"\n=== Padrão antitético \"X, não Y\": {len(achados)} ocorrências ===")
    print("Traço de escrita LLM quando usado em excesso (>5-6 no documento todo).")
    for ln, txt in achados[:15]:
        print(f"  L{ln}: {txt[:100]}")
    if len(achados) > 15:
        print(f"  ... e mais {len(achados) - 15}")


def check_adjetivos_vazios(texto: str):
    print("\n=== Adjetivos que podem estar vazios (sem número a menos de 15 chars) ===")
    total = 0
    for adj in ADJETIVOS_VAZIOS:
        for i, linha in enumerate(texto.split("\n"), 1):
            for m in re.finditer(rf"\b{adj}\b", linha, re.IGNORECASE):
                janela = linha[max(0, m.start() - 15):m.end() + 15]
                if not re.search(r"\d", janela):
                    total += 1
                    if total <= 20:
                        print(f"  L{i} [{adj}]: ...{janela}...")
    if total > 20:
        print(f"  ... e mais {total - 20}")
    print(f"  Total: {total}")


def check_negrito_abertura(texto: str):
    pattern = r"^\\textbf\{[^}]+\}|^\*\*[^*]+\*\*"
    achados = contar(pattern, texto)
    print(f"\n=== Negrito abrindo parágrafo (bullet disfarçado de prosa): {len(achados)} ===")
    for ln, txt in achados[:10]:
        print(f"  L{ln}: {txt[:90]}")


def check_conectivos_cliche(texto: str):
    print(f"\n=== Conectivos de abertura clichê: limite de referência >8 no documento ===")
    total = 0
    achados_por_termo = {}
    for termo in CONECTIVOS_CLICHE:
        achados = contar(re.escape(termo), texto, re.IGNORECASE)
        if achados:
            achados_por_termo[termo] = achados
            total += len(achados)
    for termo, achados in achados_por_termo.items():
        print(f"  \"{termo}\": {len(achados)} ocorrência(s)")
        for ln, txt in achados[:3]:
            print(f"    L{ln}: {txt[:90]}")
    print(f"  Total: {total} ({'ACIMA do limite de referência' if total > 8 else 'dentro do esperado'})")


def check_transicoes_vazias(texto: str):
    print("\n=== Transições vazias (reformulação redundante do que já foi dito) ===")
    total = 0
    for termo in TRANSICOES_VAZIAS:
        achados = contar(re.escape(termo), texto, re.IGNORECASE)
        for ln, txt in achados:
            total += 1
            print(f"  L{ln} [\"{termo}\"]: {txt[:90]}")
    if total == 0:
        print("  Nenhuma encontrada.")
    else:
        print(f"  Total: {total}")


def check_adverbios_mente(texto: str):
    palavras = re.findall(r"\b\w+\b", texto)
    n_palavras = len(palavras) or 1
    achados_mente = re.findall(r"\b\w+mente\b", texto, re.IGNORECASE)
    taxa_por_mil = len(achados_mente) / n_palavras * 1000
    print(f"\n=== Advérbios em \"-mente\": {len(achados_mente)} ocorrências "
          f"({taxa_por_mil:.1f} por 1000 palavras) ===")
    print("  Referência: acima de ~6-8/1000 palavras costuma indicar prosa inflada.")
    contagem = {}
    for a in achados_mente:
        contagem[a.lower()] = contagem.get(a.lower(), 0) + 1
    mais_comuns = sorted(contagem.items(), key=lambda x: -x[1])[:10]
    for palavra, n in mais_comuns:
        print(f"  {palavra}: {n}")


def check_adjetivos_coordenados(texto: str):
    print("\n=== Pares de adjetivos coordenados redundantes (duplicação de qualificador) ===")
    total = 0
    for pattern, label in ADJETIVOS_COORDENADOS_SUSPEITOS:
        achados = contar(pattern, texto, re.IGNORECASE)
        for ln, txt in achados:
            total += 1
            print(f"  L{ln} [{label}]: {txt[:90]}")
    if total == 0:
        print("  Nenhum encontrado.")


def check_densidade_paragrafo(texto: str, path: Path):
    """Desvio padrão do tamanho de parágrafo (em palavras) — texto gerado tende a ter
    parágrafos de tamanho muito uniforme; texto humano varia mais."""
    if path.suffix == ".tex":
        blocos = re.split(r"\n\s*\n", texto)
    else:
        blocos = re.split(r"\n\s*\n", texto)
    tamanhos = []
    for b in blocos:
        limpo = re.sub(r"\\[a-zA-Z]+(\{[^}]*\})?", " ", b)
        limpo = re.sub(r"[%].*", "", limpo)
        n = len(re.findall(r"\b\w+\b", limpo))
        if n >= 15:
            tamanhos.append(n)
    print(f"\n=== Densidade de parágrafo (uniformidade de tamanho) ===")
    if len(tamanhos) < 3:
        print("  Poucos parágrafos substanciais para calcular variação.")
        return
    media = sum(tamanhos) / len(tamanhos)
    variancia = sum((t - media) ** 2 for t in tamanhos) / len(tamanhos)
    desvio = variancia ** 0.5
    cv = desvio / media if media else 0
    print(f"  {len(tamanhos)} parágrafos substanciais (>=15 palavras); "
          f"média={media:.0f} palavras, desvio-padrão={desvio:.0f}, "
          f"coeficiente de variação={cv:.2f}")
    print("  Referência: CV < 0.35 sugere parágrafos artificialmente uniformes "
          "(texto humano bem escrito costuma variar mais).")


def check_meta_comentario(texto: str):
    termos = ["nesta correção", "desta correção", "versão revisada", "foi retirada",
              "nesta versão", "auditoria desta", "ao reconferir"]
    print("\n=== Meta-comentário sobre o processo de correção (vazamento editorial) ===")
    total = 0
    for termo in termos:
        achados = contar(re.escape(termo), texto, re.IGNORECASE)
        for ln, txt in achados:
            total += 1
            print(f"  L{ln} [\"{termo}\"]: {txt[:90]}")
    if total == 0:
        print("  Nenhum encontrado.")


def check_citacoes_orfas_tex(texto: str, path: Path):
    """Só faz sentido em .tex: citação no corpo sem \\bibitem, ou \\bibitem nunca citado."""
    if path.suffix != ".tex":
        return
    citados_raw = re.findall(r"\\cite\{([^}]+)\}", texto)
    citados = set()
    for grupo in citados_raw:
        for chave in grupo.split(","):
            citados.add(chave.strip())
    bibitems = set(re.findall(r"\\bibitem\{([^}]+)\}", texto))
    orfas_no_corpo = citados - bibitems
    decorativas = bibitems - citados
    print(f"\n=== Bibliografia: {len(bibitems)} \\bibitem, {len(citados)} chaves \\cite ===")
    if orfas_no_corpo:
        print(f"  \\cite sem \\bibitem correspondente ({len(orfas_no_corpo)}): {sorted(orfas_no_corpo)}")
    if decorativas:
        print(f"  \\bibitem nunca citado no corpo, referência decorativa ({len(decorativas)}): {sorted(decorativas)}")
    if not orfas_no_corpo and not decorativas:
        print("  OK: toda referência é citada e toda citação existe na bibliografia.")


def check_hardware(texto: str):
    tem_cpu_generico = bool(re.search(r"\bem CPU\b", texto))
    tem_hardware_especifico = bool(re.search(
        r"\b(Intel|AMD|Ryzen|Core i\d|Xeon)\b|\d+(?:[.,]\d+)?\s*GHz\b|\d+\s*núcleos\b", texto))
    print("\n=== Declaração de hardware ===")
    if tem_cpu_generico and not tem_hardware_especifico:
        print("  FALTA: menciona 'em CPU' mas nunca especifica modelo/GHz/núcleos.")
        print("  Sem isso, benchmarks de tempo não são cientificamente defensáveis.")
    elif tem_hardware_especifico:
        print("  OK: hardware específico encontrado no texto.")
    else:
        print("  AVISO: nenhuma menção a CPU/hardware encontrada. Se há benchmark de")
        print("  tempo de execução, declarar hardware é obrigatório para reprodutibilidade.")


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)
    path = Path(sys.argv[1])
    if not path.exists():
        print(f"Arquivo não encontrado: {path}", file=sys.stderr)
        sys.exit(1)
    texto = path.read_text(encoding="utf-8")

    print(f"Auditoria determinística de: {path}")
    print("=" * 60)
    check_travessao(texto)
    check_antitese(texto)
    check_adjetivos_vazios(texto)
    check_adverbios_mente(texto)
    check_conectivos_cliche(texto)
    check_transicoes_vazias(texto)
    check_adjetivos_coordenados(texto)
    check_densidade_paragrafo(texto, path)
    check_negrito_abertura(texto)
    check_meta_comentario(texto)
    check_citacoes_orfas_tex(texto, path)
    check_hardware(texto)
    print("\n" + "=" * 60)
    print("Isto é um detector, não um veredito. Cada ocorrência pode ser legítima;")
    print("julgamento de manter/cortar é de quem revisa, não deste script.")


if __name__ == "__main__":
    main()
