call plug#begin('~/.local/share/nvim/plugged')

Plug 'preservim/nerdtree'
Plug 'junegunn/fzf', { 'do': './install --all' }
Plug 'junegunn/fzf.vim'
Plug 'airblade/vim-gitgutter'
Plug 'kdheepak/lazygit.nvim'

call plug#end()

let mapleader = " "
			
" ショートカットキー設定（Ctrl-eでNERDTreeをトグル）
nnoremap <C-e> :NERDTreeToggle<CR>
nnoremap <leader>ff :Files<CR>
nnoremap <leader>rg :Rg<CR>
nnoremap <leader>bl :Buffers<CR>
nnoremap <leader>hl :History<CR>
nnoremap <leader>lg :terminal lazygit<CR>
nnoremap <leader>gs :!git status<CR>
nnoremap <leader>gc :!git commit<CR>
nnoremap <leader>gl :!git log<CR>
nnoremap <leader>gd :!git diff<CR>
