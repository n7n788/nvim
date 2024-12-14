call plug#begin('~/.local/share/nvim/plugged')

Plug 'preservim/nerdtree'
Plug 'junegunn/fzf', { 'do': './install --all' }
Plug 'junegunn/fzf.vim'
Plug 'airblade/vim-gitgutter'
Plug 'kdheepak/lazygit.nvim'

call plug#end()

let mapleader = " " " leaderキーの設定
let g:gitgutter_highlight_lines = 1 " gitでコード差分に色をつける			
set number " 行番号を付与

" NERDをディレクトリ構造変更時に自動でリフレッシュ
autocmd BufWritePost * if exists("t:NERDTreeBufName") | execute 'NERDTreeRefreshRoot' | endif

" ショートカットキー設定
tnoremap <ESC>t <C-\><C-n>
nnoremap <C-h> <C-w>h
nnoremap <C-j> <C-w>j
nnoremap <C-k> <C-w>k
nnoremap <C-l> <C-w>l
nnoremap <leader>e :NERDTreeToggle<CR>
nnoremap <leader>ff :Files<CR>
nnoremap <leader>rg :Rg<CR>
nnoremap <leader>bl :Buffers<CR>
nnoremap <leader>hl :History<CR>
nnoremap <leader>ll :terminal lazygit<CR>
