.PHONY: figures pdf clean

figures:
	python3 generate_figures.py

pdf: figures
	pdflatex -interaction=nonstopmode main.tex
	pdflatex -interaction=nonstopmode main.tex

clean:
	rm -f main.aux main.log main.out main.toc main.pdf
