/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strrchr.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/11/17 14:31:04 by youskim           #+#    #+#             */
/*   Updated: 2021/11/29 21:00:57 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

char	*ft_strrchr(const char *str, int c)
{
	char	k;
	char	*ptr;
	int		i;

	k = (char)c;
	i = 0;
	ptr = 0;
	while (str[i] != '\0')
	{
		if (str[i] == k)
			ptr = (char *)&str[i];
		i++;
	}
	if (c == 0)
		return ((char *)&str[i]);
	return (ptr);
}
