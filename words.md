---
layout: page
title: Word Index
permalink: /words/
order: 99
---

This page contains all the words in Ein and their help text.  You can
access this within Ein by running <br>`( word ) help`.  Note that
different compiles of Ein (for different robots) will contain
different words.  For example, if you have compiled Ein for Jaco, then
the Baxter-specific words will not appear in this index, and
vice-versa if you have compiled Ein for Baxter and not Jaco. 

<style>

table {
  table-layout: fixed;
  width: 100%;
}
td {
  padding: 10px;
  border: solid 1px #000;
}
tr td:first-child {
  overflow: auto;
  white-space: normal;
  width: 10%;
}
</style>

{% include ein_words.html %}
