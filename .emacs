
;; Added by Package.el.  This must come before configurations of
;; installed packages.  Don't delete this line.  If you don't want it,
;; just comment it out by adding a semicolon to the start of the line.
;; You may delete these explanatory comments.


;; getting the melpa package manager --yay--
(require 'package)
(package-initialize)
(add-to-list 'package-archives '("melpa" . "http://melpa.milkbox.net/packages/") t)


;;Standard jedi settings
(add-hook 'python-mode-hook 'jedi:setup)
(setq jedi:complete-on-dot t)

;;(require 'js2-mode)
;;(add-to-list 'auto-mode-alist '("\\.js\\'" . js2-mode))

;;js2 mode
;;(add-hook js2-mode-hook #'js2-imenu-extras-mode)

;;enable the flycheck mode
(add-hook 'after-init-hook #'global-flycheck-mode)


(custom-set-variables
 ;; custom-set-variables was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 '(ansi-color-faces-vector
   [default default default italic underline success warning error])
 '(column-number-mode t)
 '(custom-enabled-themes (quote (tango-dark)))
 '(line-number-mode nil)
 '(package-selected-packages
   (quote
    (drupal-spell flylisp jumpc js2-mode yasnippet yascroll jedi-direx auto-yasnippet flycheck jedi)))
 '(semantic-mode t))
 (global-linum-mode t)
;;(load-file "C:\\Users\\jee11\\emacs-25.2-x86_64\\emacs-for-python-master\\epy-init.el")

(custom-set-faces
 ;; custom-set-faces was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 )
