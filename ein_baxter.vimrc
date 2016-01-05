
" Commented lines start with " as you can see.
" Syntax hilighting is absolutely essential.
syntax on


" This adjusts the tab structure.  It is functional for me but probably 
" needs to be fixed in order to behave ENTIRELY correctly.
set shiftwidth=2
set softtabstop=2

" Display line numbers.
set nu

" This option hilights your searches.  To undo this after a search, type :nohls
set hlsearch

let g:lisp_rainbow = 1

"colorscheme custom
"colorscheme nightflight

au BufNewFile,BufRead *.back set syntax=lisp



