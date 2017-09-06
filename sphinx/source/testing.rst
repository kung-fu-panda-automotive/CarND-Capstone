.. highlight:: rest

Testing
========

Section
---------

Subsection
^^^^^^^^^^^^

* To create notes

    .. rst:directive:: .. note::

.. note:: 

    this is a note


* to create warnings

    .. rst:directive:: .. warning::

    .. warning::
        this is a warning


* To create version specific information

    .. rst:directive:: .. versionadded:: version

        .. versionadded:: 2.5
            The *spam* parameter.

    .. rst:directive:: .. versionchanged:: version

        .. versionchanged:: 1.0
            How *info* is printed on screen

    .. rst:directive:: .. deprecated:: version

        .. deprecated:: 3.1
            Use :func:`spam` instead.

* To create a reference Section

    .. rst:directive:: seealso

    .. seealso::

        Module :py:mod:`zipfile`
            Documentation of the :py:mod:`zipfile` standard module.

        `GNU tar manual, Basic Tar Format <http://link>`_
            Documentation for tar archive files, including GNU tar extensions.

* To create a paragraph heading witch is not used in table of contents

    .. rst:directive:: .. rubric:: title

        .. rubric:: New Title 

* Create a compact bullet list

    .. rst:directive:: hlist

        .. hlist::
         :columns: 3

         * A list of
         * short items
         * that should be
         * displayed
         * horizontally
         * in 3 columns

* Create a glossary

    .. rst:directive:: .. glossary::

    .. glossary::

         environment
            A structure where information about all documents under the root is
            saved, and used for cross-referencing.  The environment is pickled
            after the parsing stage, so that successive runs only need to read
            and parse new and changed documents.

         source directory
            The directory which, including its subdirectories, contains all
            source files for one Sphinx project.

* Code highlighting

    * For whole documents

        .. rst:directive:: .. highlight:: language

        Example::

            .. highlight:: python

    * For a code block

        .. rst:directive:: .. code-block:: language

        .. code-block:: python

            def function(x, y):
                print('hi', x, y)

    * valid options for language are 

        * ``none`` (no highlighting)
        * ``python`` (the default)
        * ``guess`` (let Pygments guess the lexer based on contents, only works with
            certain well-recognizable languages)
        * ``rest``
        * ``c``
        * ... and any other lexer alias that Pygments supports
            `<http://pygments.org/docs/lexers/>`_.

    * line numbers

        *  :: :linenothreshold: 5 

            This will produce line numbers for all code blocks longer than five lines.

        *  :: :linenos: 

            Show line numbers

        *  :: :lineno-start: 10

            Start line numbering from 10

        *  :: :emphasize-lines: 3,5

            This will emphasize lines 3 and 5


    * caption and names

        For example::

            .. code-block:: python
                :caption: this.py
                :name: this-py

                print 'Explicit is better than implicit.'

        will give

        .. code-block:: python
            :caption: this.py
            :name: this-py

            print 'Explicit is better than implicit.'



* Creating Links

    * Simple External links :: ```<http://www.python.org/>`_``

        `<http://www.python.org/>`_

    * Simple External Links with Name :: ```Python <http://www.python.org/>`_``

        `Python <http://www.python.org/>`_

    * Implicit Links to titles :: ````Subsection`_``

        `Subsection`_

    * Explicit Links

        You can create a link to an rst file by using :: ``.. _some_name:`` and then refer with
        :: ``some_name_`` or ``:ref:`some_name```. The first works ONLY for references **inside** the rst file
        and the second is a cross reference option

        :ref:`dbwNode`



.. index::
    single: execution; context
    module: __main__
    module: sys
    triple: module; search; path
