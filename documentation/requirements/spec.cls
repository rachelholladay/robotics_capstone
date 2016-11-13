\ProvidesClass{spec}[2010/07/10 v0.9 Resume class]
\LoadClass[11pt,letterpaper]{article} % Font size and paper type

\usepackage[parfill]{parskip} % Remove paragraph indentation
\usepackage{array} % Required for boldface (\bf and \bfseries) tabular columns
\usepackage{ifthen} % Required for ifthenelse statements

%----------------------------------------------------------------------------------------
%	REQUIREMENTS FORMATTING
%---------------------------------------------------------------------------------------

\newcounter{myfrctr}
\newenvironment{functional_requirement}[2]{ % 2 input arguments - name, priority
 \refstepcounter{myfrctr} % increment the environment's counter
 {\bf FR\themyfrctr : #1} \hfill {Priority #2} % Bold company name and date on the right
  \smallskip
  \begin{list}{$\cdot$}{\leftmargin=0em} % \cdot used for bullets, no indentation
   \itemsep -0.5em \vspace{-0.5em} % Compress items in list together for aesthetics
  }{
  \end{list}
  \vspace{0.5em} % Some space after the list of bullet points
}

\newcounter{mynfrctr}
\newenvironment{nonfunctional_requirement}[2]{ % 2 input arguments - name, priority
 \refstepcounter{mynfrctr} % increment the environment's counter
 {\bf NFR\themynfrctr : #1} \hfill {Priority #2} % Bold company name and date on the right
  \smallskip
  \begin{list}{$\cdot$}{\leftmargin=0em} % \cdot used for bullets, no indentation
   \itemsep -0.5em \vspace{-0.5em} % Compress items in list together for aesthetics
  }{
  \end{list}
  \vspace{0.5em} % Some space after the list of bullet points
}
